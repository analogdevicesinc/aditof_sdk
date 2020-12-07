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
#include "camera_fx1.h"

#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/frame_operations.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <glog/logging.h>
#include <iterator>
#include <map>
#include <math.h>

struct rangeStruct {
    std::string mode;
    int minDepth;
    int maxDepth;
};

// A map that contains the specific values for each revision
static const std::map<std::string, std::array<rangeStruct, 3>>
    RangeValuesForRevision = {
        {"RevA", {{{"near", 250, 800}, {"medium", 300, 3000}}}}
};

static const std::string skCustomMode = "custom";

static const std::vector<std::string> availableControls = {
    "noise_reduction_threshold", "ir_gamma_correction"};

CameraFx1::CameraFx1(std::unique_ptr<aditof::DeviceInterface> device)
    : m_device(std::move(device)), m_devStarted(false),
      m_devProgrammed(false), m_availableControls(availableControls),
      m_revision("RevA") {}

CameraFx1::~CameraFx1() = default;

aditof::Status CameraFx1::initialize() {
    using namespace aditof;

    LOG(INFO) << "Initializing camera";

    Status status = m_device->open();

    if (status != Status::OK) {
        LOG(WARNING) << "Failed to open device";
        return status;
    }

    status = m_calibration.readCalMap(m_device);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read calibration data from eeprom";
        return status;
    }

    m_details.bitCount = 12;

    LOG(INFO) << "Camera initialized";

    m_calibration.getIntrinsic(INTRINSIC, m_details.intrinsics.cameraMatrix);
    m_calibration.getIntrinsic(DISTORTION_COEFFICIENTS,
                               m_details.intrinsics.distCoeffs);

    // For now we use the unit cell size values specified in the datasheet
    m_details.intrinsics.pixelWidth = 0.0056;
    m_details.intrinsics.pixelHeight = 0.0056;
    return Status::OK;
}

aditof::Status CameraFx1::start() {
    // return m_device->start(); // For now we keep the device open all the time
    return aditof::Status::OK;
}

aditof::Status CameraFx1::stop() {
    // return m_device->stop(); // For now we keep the device open all the time
    return aditof::Status::OK;
}

aditof::Status CameraFx1::setMode(const std::string &mode,
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
        m_details.maxDepth = (*iter).maxDepth;
        m_details.minDepth = (*iter).minDepth;
    } else {
        m_details.maxDepth = 1;
    }

    LOG(INFO) << "Camera range for mode: " << mode
              << " is: " << m_details.minDepth << " mm and "
              << m_details.maxDepth << " mm";


    if(!m_devProgrammed) {
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
        status = m_device->program((uint8_t *)firmwareData.data(),
                                   2 * firmwareData.size());
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to program AFE";
            return Status::UNREACHABLE;
        }
        m_devProgrammed = true;
    }

    status = m_calibration.setMode(m_device, mode,
                                   m_details.maxDepth,
                                   m_details.frameType.width,
                                   m_details.frameType.height);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to set calibration mode";
        return status;
    }

    // register writes for enabling only one video stream (depth/ ir)
    // must be done here after programming the camera in order for them to
    // work properly. Setting the mode of the camera, programming it
    // with a different firmware would reset the value in the 0xc3da register
    if (m_details.frameType.type == "depth_only") {
        uint16_t afeRegsAddr[5] = {0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22};
        uint16_t afeRegsVal[5] = {0x0006, 0x0004, 0x03, 0x0007, 0x0004};
        m_device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
    } else if (m_details.frameType.type == "ir_only") {
        uint16_t afeRegsAddr[5] = {0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22};
        uint16_t afeRegsVal[5] = {0x0006, 0x0004, 0x05, 0x0007, 0x0004};
        m_device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
    }

    m_details.mode = mode;

    return status;
}

aditof::Status CameraFx1::getAvailableModes(
    std::vector<std::string> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;

    // Dummy data. To remove when implementig this method
    availableModes.emplace_back("near");
    availableModes.emplace_back("medium");

    return status;
}

aditof::Status CameraFx1::setFrameType(const std::string &frameType) {
    using namespace aditof;
    Status status = Status::OK;

    std::vector<FrameDetails> detailsList;
    status = m_device->getAvailableFrameTypes(detailsList);
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
        status = m_device->setFrameType(*frameDetailsIt);
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to set frame type";
            return status;
        }
        m_details.frameType = *frameDetailsIt;
    }

    if (!m_devStarted) {
        status = m_device->start();
        if (status != Status::OK) {
            return status;
        }
        m_devStarted = true;
    }

    return status;
}

aditof::Status CameraFx1::getAvailableFrameTypes(
    std::vector<std::string> &availableFrameTypes) const {
    using namespace aditof;
    Status status = Status::OK;

    std::vector<FrameDetails> frameDetailsList;
    status = m_device->getAvailableFrameTypes(frameDetailsList);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get available frame types";
        return status;
    }

    for (const auto &item : frameDetailsList) {
        availableFrameTypes.emplace_back(item.type);
    }

    return status;
}

aditof::Status CameraFx1::requestFrame(aditof::Frame *frame,
                                          aditof::FrameUpdateCallback /*cb*/) {
    using namespace aditof;
    Status status = Status::OK;

    FrameDetails frameDetails;
    frame->getDetails(frameDetails);

    if (m_details.frameType != frameDetails) {
        frame->setDetails(m_details.frameType);
    }

    uint16_t *frameDataLocation;
    frame->getData(FrameDataType::RAW, &frameDataLocation);

    status = m_device->getFrame(frameDataLocation);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get frame from device";
        return status;
    }

    if (m_details.mode != skCustomMode &&
        (m_details.frameType.type == "depth_ir" ||
         m_details.frameType.type == "depth_only")) {

        m_calibration.calibrateCameraGeometry(
            frameDataLocation,
            m_details.frameType.width * m_details.frameType.height / 2);
    }

    return Status::OK;
}

aditof::Status CameraFx1::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

std::shared_ptr<aditof::DeviceInterface> CameraFx1::getDevice() {
    return m_device;
}

aditof::Status
CameraFx1::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;
    Status status = Status::OK;

    controls = m_availableControls;

    return status;
}

aditof::Status CameraFx1::setControl(const std::string &control,
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

    return status;
}

aditof::Status CameraFx1::getControl(const std::string &control,
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

    return status;
}

aditof::Status CameraFx1::setNoiseReductionTreshold(uint16_t treshold) {
    using namespace aditof;

    const size_t REGS_CNT = 5;
    uint16_t afeRegsAddr[REGS_CNT] = {0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22};
    uint16_t afeRegsVal[REGS_CNT] = {0x0006, 0x0004, 0x8000, 0x0007, 0x0004};

    afeRegsVal[2] |= treshold;
    m_noiseReductionThreshold = treshold;

    return m_device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
}

aditof::Status CameraFx1::setIrGammaCorrection(float gamma) {
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

    status = m_device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 8);
    if (status != Status::OK) {
        return status;
    }
    status = m_device->writeAfeRegisters(afeRegsAddr + 8, afeRegsVal + 8, 8);
    if (status != Status::OK) {
        return status;
    }

    m_irGammaCorrection = gamma;

    return status;
}
