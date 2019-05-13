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

aditof::Status UsbDevice::program(const char *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::getFrame(unsigned short *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::readEeprom(unsigned short address, char *data,
                                     size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::writeEeprom(unsigned short address, const char *data,
                                      size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::readAfeRegisters(unsigned short *address,
                                           unsigned short *data,
                                           size_t lenght) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDevice::writeAfeRegisters(unsigned short *address,
                                            const unsigned short *value,
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
