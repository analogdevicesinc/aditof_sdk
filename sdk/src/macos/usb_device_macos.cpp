#include "usb_device.h"

struct UsbDevice::ImplData {};

UsbDevice::UsbDevice(const aditof::DeviceConstructionData & /*data*/) {}

UsbDevice::~UsbDevice() = default;

aditof::Status UsbDevice::open() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::start() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::stop() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status
UsbDevice::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::readEeprom(uint32_t address, uint8_t *data,
                                     size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::writeEeprom(uint32_t address, const uint8_t *data,
                                      size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::readAfeRegisters(const uint16_t *address,
                                           uint16_t *data, size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::writeAfeRegisters(const uint16_t *address,
                                            const uint16_t *data,
                                            size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::readAfeTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::readLaserTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}
