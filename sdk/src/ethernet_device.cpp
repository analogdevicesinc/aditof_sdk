#include "ethernet_device.h"

EthernetDevice::EthernetDevice(const aditof::DeviceConstructionData &data) {}

aditof::Status EthernetDevice::open() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::start() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::stop() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::getAvailableFrameTypes(
    std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status
EthernetDevice::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::readEeprom(uint32_t address, uint8_t *data,
                                          size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::writeEeprom(uint32_t address,
                                           const uint8_t *data, size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::readAfeRegisters(const uint16_t *address,
                                                uint16_t *data, size_t lenght) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::writeAfeRegisters(const uint16_t *address,
                                                 const uint16_t *data,
                                                 size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::readAfeTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::readLaserTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status EthernetDevice::setCalibrationParams(const std::string &mode,
                                                    float gain, float offset) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status
EthernetDevice::applyCalibrationToFrame(uint16_t *frame,
                                        const std::string &mode) {
    return aditof::Status::GENERIC_ERROR;
}
