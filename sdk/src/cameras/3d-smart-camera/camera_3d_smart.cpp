/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "camera_3d_smart.h"
#include "sensor_names.h"

#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/frame_operations.h>

#include <algorithm>
#include <array>
#include <cstring>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <map>
#include <math.h>

static const std::string skCameraName = "3D-Smart-Camera";

struct rangeStruct {
    std::string mode;
    int minDepth;
    int maxDepth;
};

// A map that contains the specific values for each revision
static const std::map<std::string, std::array<rangeStruct, 3>>
    RangeValuesForRevision = {
        {"RevA", {{{"near", 250, 800}, {"medium", 300, 3000}}}}};

static const std::string skCustomMode = "custom";

static const std::vector<std::string> availableControls = {

    "noise_reduction_threshold", "ir_gamma_correction",
    "depth_correction",          "camera_geometry_correction",
    "bayer_rgb_conversion",      "camera_distortion_correction",
    "ir_distortion_correction",  "revision"};
//one sensor constructor
Camera3D_Smart::Camera3D_Smart(
    std::shared_ptr<aditof::DepthSensorInterface> rgbdSensor,
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &tSensors)
    : m_rgbdSensor(rgbdSensor), m_devStarted(false), m_devProgrammed(false),
      m_eepromInitialized(false), m_tempSensorsInitialized(false),
      m_availableControls(availableControls), m_depthCorrection(true),
      m_cameraGeometryCorrection(true), m_cameraDistortionCorrection(true),
      m_irDistorsionCorrection(false), m_revision("RevA") {

    m_Rw = 255.0 * 0.25;
    m_Gw = 255.0 * 0.35;
    m_Bw = 255.0 * 0.25;

    // Check Depth Sensor
    if (!rgbdSensor) {
        LOG(WARNING) << "Invalid instance of a depth sensor";
        return;
    }
    aditof::SensorDetails sDetails;
    m_rgbdSensor->getDetails(sDetails);
    m_details.connection = sDetails.connectionType;

    // Look for EEPROM
    auto eeprom_iter =
        std::find_if(eeproms.begin(), eeproms.end(),
                     [](std::shared_ptr<aditof::StorageInterface> e) {
                         std::string name;
                         e->getName(name);
                         return name == EEPROM_NAME;
                     });
    if (eeprom_iter == eeproms.end()) {
        LOG(WARNING) << "Could not find " << EEPROM_NAME
                     << " while looking for storage for camera "
                     << skCameraName;
        return;
    }
    m_eeprom = *eeprom_iter;

    // Look for the temperature sensor
    auto tempSensorIter = std::find_if(
        tSensors.begin(), tSensors.end(),
        [](std::shared_ptr<aditof::TemperatureSensorInterface> tSensor) {
            std::string name;
            tSensor->getName(name);
            return name == TEMPERATURE_SENSOR_NAME;
        });
    if (tempSensorIter == tSensors.end()) {
        LOG(WARNING) << "Could not find " << TEMPERATURE_SENSOR_NAME
                     << " while looking for temperature sensors for "
                        "camera"
                     << skCameraName;
        return;
    }
    m_temperatureSensor = *tempSensorIter;
}

Camera3D_Smart::~Camera3D_Smart() {

    if (m_eepromInitialized) {
        m_eeprom->close();
    }
    if (m_tempSensorsInitialized) {
        m_temperatureSensor->close();
    }
}

aditof::Status Camera3D_Smart::initialize() {
    using namespace aditof;
    Status status;

    LOG(INFO) << "Initializing camera";

    if (!m_rgbdSensor || !m_eeprom || !m_temperatureSensor) {

        LOG(WARNING) << "Failed to initialize! Not all sensors are available";
        return Status::GENERIC_ERROR;
    }

    // Open communication with the depth sensor
    status = m_rgbdSensor->open();
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to open device";
        return status;
    }

    void *handle;
    status = m_rgbdSensor->getHandle(&handle);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to obtain the handle";
        return status;
    }

    m_details.bitCount = 12;

    // Open communication with EEPROM
    status = m_eeprom->open(handle);
    if (status != Status::OK) {
        std::string name;
        m_eeprom->getName(name);
        LOG(ERROR) << "Failed to open EEPROM with name " << name;
        return status;
    }
    m_eepromInitialized = true;

    // Open communication with temperature sensor
    m_temperatureSensor->open(handle);
    if (status != Status::OK) {
        std::string name;
        m_temperatureSensor->getName(name);
        LOG(ERROR) << "Failed to open temperature sensor with name " << name;
        return status;
    }
    m_tempSensorsInitialized = true;

    // Read the camera's serial number from eeprom
    uint8_t eepromSerial[EEPROM_SERIAL_LENGHT];
    status =
        m_eeprom->read(EEPROM_SERIAL_ADDR, eepromSerial, EEPROM_SERIAL_LENGHT);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read serial from eeprom";
        return status;
    }

    m_details.cameraId =
        std::string((char *)(eepromSerial), EEPROM_SERIAL_LENGHT);
    LOG(INFO) << "Camera ID: " << m_details.cameraId;

    status = m_calibration.readCalMap(m_eeprom);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read calibration data from eeprom";
        return status;
    }

    m_calibration.getIntrinsic(INTRINSIC, m_details.intrinsics.cameraMatrix);
    m_calibration.getIntrinsic(DISTORTION_COEFFICIENTS,
                               m_details.intrinsics.distCoeffs);

    // For now we use the unit cell size values specified in the datasheet
    m_details.intrinsics.pixelWidth = 0.0056f;
    m_details.intrinsics.pixelHeight = 0.0056f;

    // Cache the frame types provided by Depth and RGB sensors
    status = m_rgbdSensor->getAvailableFrameTypes(m_rgbdFrameTypes);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get the depth sensor frame types";
        return status;
    }

    LOG(INFO) << "Camera initialized";

    return Status::OK;
}

aditof::Status Camera3D_Smart::start() {
    aditof::Status status;

    status = m_rgbdSensor->start();
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to start the rgbd sensor";
        return status;
    }
    m_devStarted = true;
    return aditof::Status::OK;
}

aditof::Status Camera3D_Smart::stop() {
    aditof::Status status;

    status = m_rgbdSensor->stop();
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to stop the rgbd sensor";
        return status;
    }
    m_devStarted = false;

    return aditof::Status::OK;
}

aditof::Status Camera3D_Smart::setMode(const std::string &mode,
                                       const std::string &modeFilename) {
    using namespace aditof;
    Status status = Status::OK;

    // Set the values specific to the Revision requested
    std::array<rangeStruct, 3> rangeValues =
        RangeValuesForRevision.at(m_revision);

    LOG(INFO) << "Chosen mode: " << mode.c_str();

    auto iter = std::find_if(rangeValues.begin(), rangeValues.end(),
                             [&mode](struct rangeStruct rangeMode) {
                                 return rangeMode.mode == mode;
                             });
    if (iter != rangeValues.end()) {
        m_details.depthParameters.maxDepth = (*iter).maxDepth;
        m_details.depthParameters.minDepth = (*iter).minDepth;
    } else {
        m_details.depthParameters.maxDepth = 1;
    }

    LOG(INFO) << "Camera range for mode: " << mode
              << " is: " << m_details.depthParameters.minDepth << " mm and "
              << m_details.depthParameters.maxDepth << " mm";

    if (!m_devProgrammed) {
        std::vector<uint16_t> firmwareData;
        status = m_calibration.getAfeFirmware(mode, firmwareData);
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to read firmware from eeprom";
            return Status::UNREACHABLE;
        } else {
            LOG(INFO) << "Found firmware for mode: " << mode;
        }

        LOG(INFO) << "Firmware size: " << firmwareData.size() * sizeof(uint16_t)
                  << " bytes";
        status = m_rgbdSensor->program((uint8_t *)firmwareData.data(),
                                       2 * firmwareData.size());
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to program AFE";
            return Status::UNREACHABLE;
        }
        m_devProgrammed = true;
    }

    status = m_calibration.setMode(
        m_rgbdSensor, mode, m_details.depthParameters.maxDepth,
        m_details.frameType.width, m_details.frameType.height);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to set calibration mode";
        return status;
    }

    // register writes for enabling only one video stream (depth/ ir)
    // must be done here after programming the camera in order for them to
    // work properly. Setting the mode of the camera, programming it
    // with a different firmware would reset the value in the 0xc3da register
    if (m_details.frameType.type == "depth_rgb") {
        uint16_t afeRegsAddr[5] = {0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22};
        uint16_t afeRegsVal[5] = {0x0006, 0x0004, 0x03, 0x0007, 0x0004};
        m_rgbdSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
    } else if (m_details.frameType.type == "ir_rgb") {
        uint16_t afeRegsAddr[5] = {0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22};
        uint16_t afeRegsVal[5] = {0x0006, 0x0004, 0x05, 0x0007, 0x0004};
        m_rgbdSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
    } else if (m_details.frameType.type == "depth_ir_rgb") {
        uint16_t afeRegsAddr[5] = {0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22};
        uint16_t afeRegsVal[5] = {0x0006, 0x0004, 0x03, 0x0007, 0x0004};

        m_rgbdSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
    }

    m_details.depthParameters.depthGain = (mode == "near" ? 0.5f : 1.15f);
    m_details.depthParameters.depthOffset = 0.0f;

    m_details.mode = mode;

    return status;
}

aditof::Status Camera3D_Smart::getAvailableModes(
    std::vector<std::string> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;

    // Dummy data. To remove when implementig this method
    availableModes.emplace_back("near");
    availableModes.emplace_back("medium");

    return status;
}

aditof::Status Camera3D_Smart::setFrameType(const std::string &frameType) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_devStarted) {
        status = m_rgbdSensor->stop();
        if (status != Status::OK) {
            return status;
        }
        m_devStarted = false;
    }

    std::vector<FrameDetails> detailsList;
    status = m_rgbdSensor->getAvailableFrameTypes(detailsList);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get available frame types";
        return status;
    }

    auto frameDetailsIt = std::find_if(
        detailsList.begin(), detailsList.end(),
        [&frameType](const FrameDetails &d) { return (d.type == frameType); });

    if (frameDetailsIt == detailsList.end()) {
        LOG(WARNING) << "Frame type: " << frameType
                     << " not supported by camera";
        return Status::INVALID_ARGUMENT;
    }

    if (m_details.frameType != *frameDetailsIt) {

        //Turn off the streaming of data to modify parameters in the driver.
        uint16_t afeRegsAddr[2] = {0x4001, 0x7c22};
        uint16_t afeRegsVal[2] = {0x0006, 0x0004};
        m_rgbdSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 2);

        status = m_rgbdSensor->setFrameType(*frameDetailsIt);

        if (status != Status::OK) {
            LOG(WARNING) << "Failed to set frame type of the depth sensor";
            return status;
        }
        m_details.frameType.type = frameType;
        m_details.frameType.width = frameDetailsIt->width;
        m_details.frameType.height = frameDetailsIt->height;
        m_details.frameType.rgbWidth = frameDetailsIt->rgbWidth;
        m_details.frameType.rgbHeight = frameDetailsIt->rgbHeight;
        m_details.frameType.fullDataWidth = frameDetailsIt->fullDataWidth;
        m_details.frameType.fullDataHeight = frameDetailsIt->fullDataHeight;

        //Turn on the streaming of data.
        uint16_t afeRegsAddress[2] = {0x4001, 0x7c22};
        uint16_t afeRegsValue[2] = {0x0007, 0x0004};
        m_rgbdSensor->writeAfeRegisters(afeRegsAddress, afeRegsValue, 2);
    }

    if (!m_devStarted) {
        status = m_rgbdSensor->start();
        if (status != Status::OK) {
            return status;
        }
        m_devStarted = true;
    }

    return status;
}

aditof::Status Camera3D_Smart::getAvailableFrameTypes(
    std::vector<std::string> &availableFrameTypes) const {
    using namespace aditof;
    Status status = Status::OK;

    availableFrameTypes.clear();
    std::vector<FrameDetails> frameDetailsList;
    status = m_rgbdSensor->getAvailableFrameTypes(frameDetailsList);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get available frame types";
        return status;
    }

    for (const auto &item : frameDetailsList) {
        availableFrameTypes.emplace_back(item.type);
    }

    return status;
}

aditof::Status
Camera3D_Smart::requestFrame(aditof::Frame *frame,
                             aditof::FrameUpdateCallback /*cb*/) {
    using namespace aditof;
    Status status = Status::OK;

    if (frame == nullptr) {
        LOG(ERROR) << "Received pointer frame is null";
        return aditof::Status::INVALID_ARGUMENT;
    }

    FrameDetails frameDetails;
    frame->getDetails(frameDetails);

    if (m_details.frameType.type != frameDetails.type) {
        frame->setDetails(m_details.frameType);
    }

    uint16_t *fullDataLocation;
    frame->getData(FrameDataType::FULL_DATA, &fullDataLocation);

    aditof::BufferInfo rgbdBufferInfo;
    status = m_rgbdSensor->getFrame(fullDataLocation, &rgbdBufferInfo);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get frame from depth sensor";
        return status;
    }

#ifdef BAYER_CONVERSION
    //conversion for bayer to rgb
    uint16_t *copyOfRgbData =
        (uint16_t *)malloc(m_details.frameType.rgbWidth *
                           m_details.frameType.rgbHeight * sizeof(uint16_t));

    std::memcpy(copyOfRgbData, rgbDataLocation,
                m_details.frameType.rgbWidth * m_details.frameType.rgbHeight *
                    2);

    bayer2RGB(rgbDataLocation, (uint8_t *)copyOfRgbData,
              m_details.frameType.rgbWidth, m_details.frameType.rgbHeight);
    free(copyOfRgbData);
}
#endif

if (m_details.mode != skCustomMode &&
    (m_details.frameType.type == "depth_ir_rgb" ||
     m_details.frameType.type == "depth_rgb")) {

    uint16_t *depthDataLocation;
    frame->getData(FrameDataType::DEPTH, &depthDataLocation);

    uint16_t *irDataLocation;
    frame->getData(FrameDataType::IR, &irDataLocation);

    if (m_depthCorrection) {
        m_calibration.calibrateDepth(depthDataLocation,
                                     m_details.frameType.width *
                                         m_details.frameType.height);
    }
    if (m_cameraGeometryCorrection) {
        m_calibration.calibrateCameraGeometry(depthDataLocation,
                                              m_details.frameType.width *
                                                  m_details.frameType.height);
    }
    if (m_cameraDistortionCorrection) {
        m_calibration.distortionCorrection(depthDataLocation,
                                           m_details.frameType.width,
                                           m_details.frameType.height);
    }
    if (m_irDistorsionCorrection) {
        m_calibration.distortionCorrection(irDataLocation,
                                           m_details.frameType.width,
                                           m_details.frameType.height);
    }
}

return Status::OK;
}

aditof::Status
Camera3D_Smart::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

aditof::Status Camera3D_Smart::getImageSensors(
    std::vector<std::shared_ptr<aditof::DepthSensorInterface>> &sensors) {
    sensors.clear();
    sensors.emplace_back(m_rgbdSensor);

    return aditof::Status::OK;
}

aditof::Status Camera3D_Smart::getEeproms(
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms) {
    eeproms.clear();
    eeproms.push_back(m_eeprom);

    return aditof::Status::OK;
}

aditof::Status Camera3D_Smart::getTemperatureSensors(
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &sensors) {
    sensors.clear();
    sensors.push_back(m_temperatureSensor);

    return aditof::Status::OK;
}

aditof::Status
Camera3D_Smart::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;
    Status status = Status::OK;

    controls = m_availableControls;

    return status;
}

aditof::Status Camera3D_Smart::setControl(const std::string &control,
                                          const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    auto it = std::find(m_availableControls.begin(), m_availableControls.end(),
                        control);
    if (it == m_availableControls.end()) {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    if (control == "noise_reduction_threshold") {
        return setNoiseReductionTreshold((uint16_t)std::stoi(value));
    }

    if (control == "ir_gamma_correction") {
        return setIrGammaCorrection(std::stof(value));
    }

    if (control == "depth_correction") {
        m_depthCorrection = std::stoi(value) != 0;
    }

    if (control == "camera_geometry_correction") {
        m_cameraGeometryCorrection = std::stoi(value) != 0;
    }

    if (control == "bayer_rgb_conversion") {
        m_cameraBayerRgbConversion = std::stoi(value) != 0;
    }

    if (control == "revision") {
        m_revision = value;
    }
    if (control == "camera_distortion_correction") {
        m_cameraDistortionCorrection = std::stoi(value) != 0;
    }
    if (control == "ir_distortion_correction") {
        m_irDistorsionCorrection = std::stoi(value) != 0;
    }
    return status;
}

aditof::Status Camera3D_Smart::getControl(const std::string &control,
                                          std::string &value) const {
    using namespace aditof;
    Status status = Status::OK;

    auto it = std::find(m_availableControls.begin(), m_availableControls.end(),
                        control);
    if (it == m_availableControls.end()) {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    if (control == "noise_reduction_threshold") {
        value = std::to_string(m_noiseReductionThreshold);
    }

    if (control == "ir_gamma_correction") {
        value = std::to_string(m_irGammaCorrection);
    }

    if (control == "depth_correction") {
        value = m_depthCorrection ? "1" : "0";
    }

    if (control == "camera_geometry_correction") {
        value = m_cameraGeometryCorrection ? "1" : "0";
    }

    if (control == "bayer_rgb_conversion") {
        value = m_cameraBayerRgbConversion ? "1" : "0";
    }
    if (control == "revision") {
        value = m_revision;
    }

    if (control == "camera_distortion_correction") {
        value = m_cameraDistortionCorrection ? "1" : "0";
    }

    if (control == "ir_distorsion_correction") {
        value = m_irDistorsionCorrection ? "1" : "0";
    }

    return status;
}

aditof::Status Camera3D_Smart::setNoiseReductionTreshold(uint16_t treshold) {
    using namespace aditof;

    if (m_details.mode.compare("far") == 0) {
        LOG(WARNING) << "Far mode does not support noise reduction!";
        return Status::UNAVAILABLE;
    }

    const size_t REGS_CNT = 5;
    uint16_t afeRegsAddr[REGS_CNT] = {0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22};
    uint16_t afeRegsVal[REGS_CNT] = {0x0006, 0x0004, 0x8000, 0x0007, 0x0004};

    afeRegsVal[2] |= treshold;
    m_noiseReductionThreshold = treshold;

    return m_rgbdSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
}

aditof::Status Camera3D_Smart::setIrGammaCorrection(float gamma) {
    using namespace aditof;
    aditof::Status status = Status::OK;
    const float x_val[] = {256, 512, 768, 896, 1024, 1536, 2048, 3072, 4096};
    uint16_t y_val[9];

    for (int i = 0; i < 9; i++) {
        y_val[i] = (uint16_t)(pow(x_val[i] / 4096.0f, gamma) * 1024.0f);
    }

    uint16_t afeRegsAddr[] = {0x4001, 0x7c22, 0xc372, 0xc373, 0xc374, 0xc375,
                              0xc376, 0xc377, 0xc378, 0xc379, 0xc37a, 0xc37b,
                              0xc37c, 0xc37d, 0x4001, 0x7c22};
    uint16_t afeRegsVal[] = {0x0006,   0x0004,   0x7888,   0xa997,
                             0x000a,   y_val[0], y_val[1], y_val[2],
                             y_val[3], y_val[4], y_val[5], y_val[6],
                             y_val[7], y_val[8], 0x0007,   0x0004};

    status = m_rgbdSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 8);
    if (status != Status::OK) {
        return status;
    }
    status =
        m_rgbdSensor->writeAfeRegisters(afeRegsAddr + 8, afeRegsVal + 8, 8);
    if (status != Status::OK) {
        return status;
    }

    m_irGammaCorrection = gamma;

    return status;
}

#ifdef BAYER_CONVERSION
float Camera3D_Smart::getValueFromData(uint8_t *pData, int x, int y, int width,
                                       int height) {
    return (float)((pData[x * width * 2 + y * 2 + 1] << 8) +
                   pData[x * width * 2 + y * 2]) /
           4095.0 * 255.0;
}

float Camera3D_Smart::verticalKernel(uint8_t *pData, int x, int y, int width,
                                     int height) {
    float sum = 0;
    int nr = 0;
    if ((x - 1) >= 0 && (x - 1) < height) {
        sum += getValueFromData(pData, x - 1, y, width, height);
        nr++;
    }
    if ((x + 1) >= 0 && (x + 1) < height) {
        sum += getValueFromData(pData, x + 1, y, width, height);
        nr++;
    }
    return (sum / (float)(nr > 0 ? nr : 1));
}
float Camera3D_Smart::horizontalKernel(uint8_t *pData, int x, int y, int width,
                                       int height) {
    float sum = 0;
    int nr = 0;
    if ((y - 1) >= 0 && (y - 1) < width) {
        sum += getValueFromData(pData, x, y - 1, width, height);
        nr++;
    }
    if ((y + 1) >= 0 && (y + 1) < width) {
        sum += getValueFromData(pData, x, y + 1, width, height);
        nr++;
    }
    return ((float)sum / (float)(nr > 0 ? nr : 1));
}
float Camera3D_Smart::plusKernel(uint8_t *pData, int x, int y, int width,
                                 int height) {
    float sum = 0;
    int nr = 0;
    if ((x - 1) >= 0 && (x - 1) < height) {
        sum += getValueFromData(pData, x - 1, y, width, height);
        nr++;
    }
    if ((x + 1) >= 0 && (x + 1) < height) {
        sum += getValueFromData(pData, x + 1, y, width, height);
        nr++;
    }
    if ((y - 1) >= 0 && (y - 1) < width) {
        sum += getValueFromData(pData, x, y - 1, width, height);
        nr++;
    }
    if ((y + 1) >= 0 && (y + 1) < width) {
        sum += getValueFromData(pData, x, y + 1, width, height);
        nr++;
    }
    return (sum / (float)(nr > 0 ? nr : 1));
}
float Camera3D_Smart::crossKernel(uint8_t *pData, int x, int y, int width,
                                  int height) {
    float sum = 0;
    int nr = 0;
    if ((x - 1) >= 0 && (x - 1) < height && (y - 1) >= 0 && (y - 1) < width) {
        sum += getValueFromData(pData, x - 1, y - 1, width, height);
        nr++;
    }
    if ((x + 1) >= 0 && (x + 1) < height && (y - 1) >= 0 && (y - 1) < width) {
        sum += getValueFromData(pData, x + 1, y - 1, width, height);
        nr++;
    }
    if ((x - 1) >= 0 && (x - 1) < height && (y + 1) >= 0 && (y + 1) < width) {
        sum += getValueFromData(pData, x - 1, y + 1, width, height);
        nr++;
    }
    if ((x + 1) >= 0 && (x + 1) < height && (y + 1) >= 0 && (y + 1) < width) {
        sum += getValueFromData(pData, x + 1, y + 1, width, height);
        nr++;
    }
    return (sum / (float)(nr > 0 ? nr : 1));
}

float Camera3D_Smart::directCopy(uint8_t *pData, int x, int y, int width,
                                 int height) {
    return (float)getValueFromData(pData, x, y, width, height); //
}

void Camera3D_Smart::bayer2RGB(uint16_t *buffer, uint8_t *pData, int width,
                               int height) {
    //casting into 8 in order to work with it
    uint8_t *rgb = (uint8_t *)buffer;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (i % 2 == RED_START_POZ_Y &&
                j % 2 == RED_START_POZ_X) //red square
            {
                rgb[(i * width + j) * 3 + RED] =
                    (uint8_t)(directCopy(pData, i, j, width, height) * 255.0 /
                              this->m_Rw);
                rgb[(i * width + j) * 3 + GREEN] = (uint8_t)(
                    plusKernel(pData, i, j, width, height) * 255.0 / m_Gw);
                rgb[(i * width + j) * 3 + BLUE] = (uint8_t)(
                    crossKernel(pData, i, j, width, height) * 255.0 / m_Bw);
            } else if (i % 2 == (RED_START_POZ_Y ^ 1) &&
                       j % 2 == (RED_START_POZ_X ^ 1)) //blue square
            {
                rgb[(i * width + j) * 3 + RED] = (uint8_t)(
                    crossKernel(pData, i, j, width, height) * 255.0 / m_Rw);
                rgb[(i * width + j) * 3 + GREEN] = (uint8_t)(
                    plusKernel(pData, i, j, width, height) * 255.0 / m_Gw);
                rgb[(i * width + j) * 3 + BLUE] = (uint8_t)(
                    directCopy(pData, i, j, width, height) * 255.0 / m_Bw);
            } else if (i % 2 == (RED_START_POZ_Y ^ 1) &&
                       j % 2 == RED_START_POZ_X) //green pixel, blue row
            {
                rgb[(i * width + j) * 3 + RED] = (uint8_t)(
                    verticalKernel(pData, i, j, width, height) * 255.0 / m_Rw);
                rgb[(i * width + j) * 3 + GREEN] = (uint8_t)(
                    directCopy(pData, i, j, width, height) * 255.0 / m_Gw);
                rgb[(i * width + j) * 3 + BLUE] =
                    (uint8_t)(horizontalKernel(pData, i, j, width, height) *
                              255.0 / m_Bw);
            } else if (i % 2 == RED_START_POZ_Y &&
                       j % 2 == (RED_START_POZ_X ^ 1)) // green pixel, red row
            {
                rgb[(i * width + j) * 3 + RED] =
                    (uint8_t)(horizontalKernel(pData, i, j, width, height) *
                              255.0 / m_Rw);
                rgb[(i * width + j) * 3 + GREEN] = (uint8_t)(
                    directCopy(pData, i, j, width, height) * 255.0 / m_Gw);
                rgb[(i * width + j) * 3 + BLUE] = (uint8_t)(
                    verticalKernel(pData, i, j, width, height) * 255.0 / m_Bw);
            }
        }
    }
}
#endif
