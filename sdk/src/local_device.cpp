/* This is an empty implementation and it's purpose is to give the LocalDevice
 * a default implementation on platforms where it is not used (Windows, Linux,
 * MacOS).
 */

#include "local_device.h"

struct LocalDevice::ImplData {};

LocalDevice::LocalDevice(const aditof::DeviceConstructionData & /*data*/) {}
LocalDevice::~LocalDevice() = default;

aditof::Status LocalDevice::open() { return aditof::Status::GENERIC_ERROR; }

aditof::Status LocalDevice::start() { return aditof::Status::GENERIC_ERROR; }

aditof::Status LocalDevice::stop() { return aditof::Status::GENERIC_ERROR; }

aditof::Status LocalDevice::getAvailableFrameTypes(
    std::vector<aditof::FrameDetails> & /*types*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status
LocalDevice::setFrameType(const aditof::FrameDetails & /*details*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::program(const uint8_t * /*firmware*/,
                                    size_t /*size*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::getFrame(uint16_t * /*buffer*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::readEeprom(uint32_t /*address*/, uint8_t * /*data*/,
                                       size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::writeEeprom(uint32_t /*address*/,
                                        const uint8_t * /*data*/,
                                        size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::readAfeRegisters(const uint16_t * /*address*/,
                                             uint16_t * /*regValue*/,
                                             size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::writeAfeRegisters(const uint16_t * /*address*/,
                                              const uint16_t * /*value*/,
                                              size_t /*length*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::readAfeTemp(float & /*temperature*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::readLaserTemp(float & /*temperature*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status LocalDevice::setCalibrationParams(const std::string & /*mode*/,
                                                 float /*gain*/,
                                                 float /*offset*/) {
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status
LocalDevice::applyCalibrationToFrame(uint16_t * /*frame*/,
                                     const std::string & /*mode*/) {
    return aditof::Status::GENERIC_ERROR;
}
