/* This is an empty implementation and it's purpose is to give the UsbDevice
 * a default implementation on platforms where it is not used (Dragonboard).
 */

#include "usb_device.h"

struct UsbDevice::ImplData {};

UsbDevice::UsbDevice(const aditof::DeviceConstructionData & /*data*/) {}

UsbDevice::~UsbDevice() = default;

aditof::Status UsbDevice::open() { return aditof::Status::GENERIC_ERROR; }

aditof::Status UsbDevice::start() { return aditof::Status::GENERIC_ERROR; }

aditof::Status UsbDevice::stop() { return aditof::Status::GENERIC_ERROR; }

aditof::Status
UsbDevice::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::setFrameType(const aditof::FrameDetails &details) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::program(const uint8_t * /*firmware*/,
                                  size_t /*size*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::getFrame(uint16_t * /*buffer*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::readEeprom(uint32_t /*address*/, uint8_t * /*data*/,
                                     size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::writeEeprom(uint32_t /*address*/,
                                      const uint8_t * /*data*/,
                                      size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::readAfeRegisters(const uint16_t * /*address*/,
                                           uint16_t * /*data*/,
                                           size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::writeAfeRegisters(const uint16_t * /*address*/,
                                            const uint16_t * /*data*/,
                                            size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::readAfeTemp(float & /*temperature*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::readLaserTemp(float & /*temperature*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status UsbDevice::setCalibrationParams(const std::string & /*mode*/,
                                               float /*gain*/,
                                               float /*offset*/) {

    return aditof::Status::GENERIC_ERROR;
}

aditof::Status
UsbDevice::applyCalibrationToFrame(uint16_t * /*frame*/,
                                   const std::string & /*mode*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status
UsbDevice::getDetails(aditof::DeviceDetails & /*details*/) const {
    return aditof::Status::GENERIC_ERROR;
}
