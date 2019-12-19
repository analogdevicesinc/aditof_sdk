#include "camera_96tof1.h"

#include <aditof/camera_96tof1_specifics.h>
#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/frame_operations.h>

#include <algorithm>
#include <fstream>
#include <glog/logging.h>
#include <iterator>

static const std::string skCustomMode = "custom";

Camera96Tof1::Camera96Tof1(std::unique_ptr<aditof::DeviceInterface> device)
    : m_specifics(std::make_shared<aditof::Camera96Tof1Specifics>(
          aditof::Camera96Tof1Specifics(this))),
      m_device(std::move(device)), m_devStarted(false) {}

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

    LOG(INFO) << "Camera initialized";

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

    LOG(INFO) << "Chosen mode: " << mode.c_str();

    std::vector<std::pair<std::string, int>> modeRanges = {
        {"near", 800}, {"medium", 4500}, {"far", 6000}};

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
        m_details.range = 4095;
    } else {
        auto iter = std::find_if(modeRanges.begin(), modeRanges.end(),
                                 [&mode](std::pair<std::string, int> mp) {
                                     return mp.first == mode;
                                 });
        if (iter != modeRanges.end()) {
            m_details.range = (*iter).second;
        } else {
            m_details.range = 1;
        }

        LOG(INFO) << "Camera range for mode: " << mode
                  << " is: " << m_details.range << " mm";

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
            auto iter = std::find_if(modeRanges.begin(), modeRanges.end(),
                                     [&mode](std::pair<std::string, int> mp) {
                                         return mp.first == mode;
                                     });
            if (iter != modeRanges.end()) {
                range = (*iter).second;
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
