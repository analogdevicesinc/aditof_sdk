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
#include "camera_chicony.h"

#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/frame_operations.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <glog/logging.h>
#include <iterator>

using namespace std;

static const std::string skCustomMode = "custom";

CameraChicony::CameraChicony(std::unique_ptr<aditof::DeviceInterface> device)
    : m_specifics(std::make_shared<aditof::CameraChiconySpecifics>(
          aditof::CameraChiconySpecifics(this))),
      m_device(std::move(device)), m_devStarted(false) {}

CameraChicony::~CameraChicony() = default;

aditof::Status CameraChicony::initialize() {
    using namespace aditof;

    LOG(INFO) << "Initializing camera";

    Status status = m_device->open();

    if (status != Status::OK) {
        LOG(WARNING) << "Failed to open device";
        return status;
    }

    m_details.bitCount = 12;

    LOG(INFO) << "Camera initialized";

    // TO DO: figure out which are the intrisics

    return Status::OK;
}

aditof::Status CameraChicony::start() {
    // return m_device->start(); // For now we keep the device open all the time
    return aditof::Status::OK;
}

aditof::Status CameraChicony::stop() {
    // return m_device->stop(); // For now we keep the device open all the time
    return aditof::Status::OK;
}

aditof::Status CameraChicony::setMode(const std::string &mode,
                                      const std::string &modeFilename) {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Chosen mode: " << mode.c_str();
    if ((mode != skCustomMode) ^ (modeFilename.empty())) {
        LOG(WARNING) << " mode must be set to: '" << skCustomMode
                     << "' and a firmware must be provided";

        return Status::INVALID_ARGUMENT;
    }

    m_details.maxDepth = 4095;
    m_details.minDepth = 0;

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
    } else {
        if (mode != "near") {
            LOG(WARNING) << "Unsupported mode";
            return Status::INVALID_ARGUMENT;
        }

        LOG(INFO) << "Camera range for mode: " << mode
                  << " is: " << m_details.minDepth << " mm and "
                  << m_details.maxDepth << " mm";

        uint32_t firmwareLength = 0;
        status = m_device->readEeprom(
            0xFFFFFFFE, reinterpret_cast<uint8_t *>(&firmwareLength),
            sizeof(firmwareLength));
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to read firmware size";
            return Status::UNREACHABLE;
        }

        uint8_t *firmwareData = new uint8_t[firmwareLength];
        status = m_device->readEeprom(0xFFFFFFFF, firmwareData, firmwareLength);

        if (status != Status::OK) {
            delete[] firmwareData;
            LOG(WARNING) << "Failed to read firmware";
            return Status::UNREACHABLE;
        } else {
            LOG(INFO) << "Found firmware for mode: " << mode;
        }

        LOG(INFO) << "Firmware size: " << firmwareLength << " bytes";
        status = m_device->program(firmwareData, firmwareLength);
        if (status != Status::OK) {
            delete[] firmwareData;
            LOG(WARNING) << "Failed to program AFE";
            return Status::UNREACHABLE;
        }

        delete[] firmwareData;
    }

    // register writes for enabling only one video stream (depth/ ir)
    // must be done here after programming the camera in order for them to
    // work properly. Setting the mode of the camera, programming it
    // with a different firmware would reset the value in the oxc3da register
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

aditof::Status CameraChicony::getAvailableModes(
    std::vector<std::string> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;

    // Dummy data. To remove when implementig this method
    availableModes.emplace_back("near");

    availableModes.emplace_back(skCustomMode);

    return status;
}

aditof::Status CameraChicony::setFrameType(const std::string &frameType) {
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

aditof::Status CameraChicony::getAvailableFrameTypes(
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

aditof::Status CameraChicony::requestFrame(aditof::Frame *frame,
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

    return Status::OK;
}

aditof::Status CameraChicony::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

std::shared_ptr<aditof::CameraSpecifics> CameraChicony::getSpecifics() {
    return m_specifics;
}

std::shared_ptr<aditof::DeviceInterface> CameraChicony::getDevice() {
    return m_device;
}
