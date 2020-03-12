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
#include "camera_96tof1.h"

#include <aditof/camera_96tof1_specifics.h>
#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/frame_operations.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <glog/logging.h>
#include <iterator>
#include <map>

struct rangeStruct {
    std::string mode;
    int minDepth;
    int maxDepth;
};

// A map that contains the specific values for each revision
static const std::map<aditof::Revision, std::array<rangeStruct, 3>>
    RangeValuesForRevision = {
        {aditof::Revision::RevB,
         {{{"near", 250, 800}, {"medium", 300, 3000}, {"far", 3000, 6000}}}},
        {aditof::Revision::RevC,
         {{{"near", 250, 800}, {"medium", 300, 4500}, {"far", 3000, 6000}}}}};

static const std::string skCustomMode = "custom";

Camera96Tof1::Camera96Tof1(std::unique_ptr<aditof::DeviceInterface> device)
    : m_specifics(std::make_shared<aditof::Camera96Tof1Specifics>(
          aditof::Camera96Tof1Specifics(this))),
      m_device(std::move(device)), m_devStarted(false) {

    // initialize range values with the default data for revision C
    auto cam96tof1Specifics =
        std::dynamic_pointer_cast<aditof::Camera96Tof1Specifics>(m_specifics);
    cam96tof1Specifics->setCameraRevision(aditof::Revision::RevC);
}

Camera96Tof1::~Camera96Tof1() = default;

aditof::Status Camera96Tof1::initialize() {
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

aditof::Status Camera96Tof1::start() {
    // return m_device->start(); // For now we keep the device open all the time
    return aditof::Status::OK;
}

aditof::Status Camera96Tof1::stop() {
    // return m_device->stop(); // For now we keep the device open all the time
    return aditof::Status::OK;
}

aditof::Status Camera96Tof1::setMode(const std::string &mode,
                                     const std::string &modeFilename) {
    using namespace aditof;
    Status status = Status::OK;

    // Set the values specific to the Revision requested
    auto cam96tof1Specifics =
        std::dynamic_pointer_cast<Camera96Tof1Specifics>(m_specifics);
    Revision revision = cam96tof1Specifics->getRevision();
    std::array<rangeStruct, 3> rangeValues =
        RangeValuesForRevision.at(revision);

    LOG(INFO) << "Chosen mode: " << mode.c_str();
    if ((mode != skCustomMode) ^ (modeFilename.empty())) {
        LOG(WARNING) << " mode must be set to: '" << skCustomMode
                     << "' and a firmware must be provided";

        return Status::INVALID_ARGUMENT;
    }

    if (!modeFilename.empty()) {
        std::ifstream firmwareFile(modeFilename.c_str(), std::ios::binary);

        if (!firmwareFile) {
            LOG(WARNING) << "Cannot find (or open) file: "
                         << modeFilename.c_str();
            return Status::UNREACHABLE;
        }

        firmwareFile.seekg(0, std::ios_base::end);
        size_t length = static_cast<size_t>(firmwareFile.tellg());
        firmwareFile.seekg(0, std::ios_base::beg);
        std::vector<uint8_t> firmwareData;
        firmwareData.reserve(length);
        std::copy(std::istreambuf_iterator<char>(firmwareFile),
                  std::istreambuf_iterator<char>(),
                  std::back_inserter(firmwareData));
        status = m_device->program(firmwareData.data(), firmwareData.size());
        firmwareFile.close();
        m_details.maxDepth = 4095;
        m_details.minDepth = 0;
    } else {
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

        status =
            m_calibration.getGainOffset(mode, m_details.frameType.cal_data.gain,
                                        m_details.frameType.cal_data.offset);
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to read gain and offset from eeprom";
            return Status::UNREACHABLE;
        } else {
            LOG(INFO) << "Camera calibration parameters for mode: " << mode
                      << " are gain: " << m_details.frameType.cal_data.gain
                      << " "
                      << "offset: " << m_details.frameType.cal_data.offset;
        }
    }

    std::vector<std::string> modes;
    getAvailableModes(modes);

    for (const std::string &mode : modes) {
        if (mode == skCustomMode)
            continue;

        float gain = 1.0, offset = 0.0;
        Status status = m_calibration.getGainOffset(mode, gain, offset);
        if (status == Status::OK) {
            int range = 1;

            auto iter = std::find_if(rangeValues.begin(), rangeValues.end(),
                                     [&mode](struct rangeStruct rangeMode) {
                                         return rangeMode.mode == mode;
                                     });

            if (iter != rangeValues.end()) {
                range = (*iter).maxDepth;
            }
            m_device->setCalibrationParams(mode, gain, offset, range);
        }
    }

    m_details.mode = mode;

    return status;
}

aditof::Status Camera96Tof1::getAvailableModes(
    std::vector<std::string> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;

    // Dummy data. To remove when implementig this method
    availableModes.emplace_back("near");
    availableModes.emplace_back("medium");
    availableModes.emplace_back("far");

    // TO DO

    availableModes.emplace_back(skCustomMode);

    return status;
}

aditof::Status Camera96Tof1::setFrameType(const std::string &frameType) {
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

aditof::Status Camera96Tof1::getAvailableFrameTypes(
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

aditof::Status Camera96Tof1::requestFrame(aditof::Frame *frame,
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
        m_details.frameType.type == "depth_ir") {
        m_device->applyCalibrationToFrame(frameDataLocation, m_details.mode);
    }

    return Status::OK;
}

aditof::Status Camera96Tof1::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

std::shared_ptr<aditof::CameraSpecifics> Camera96Tof1::getSpecifics() {
    return m_specifics;
}

std::shared_ptr<aditof::DeviceInterface> Camera96Tof1::getDevice() {
    return m_device;
}
