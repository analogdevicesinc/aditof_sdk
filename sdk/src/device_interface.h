#ifndef DEVICE_INTERFACE_H
#define DEVICE_INTERFACE_H

#include "frame_definitions.h"
#include "status_definitions.h"

#include <cstddef>
#include <vector>

class DeviceInterface {
  public:
    virtual ~DeviceInterface() = default;

    virtual aditof::Status open() = 0;
    virtual aditof::Status start() = 0;
    virtual aditof::Status stop() = 0;
    virtual aditof::Status
    getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) = 0;
    virtual aditof::Status
    setFrameType(const aditof::FrameDetails &details) = 0;
    virtual aditof::Status program(const uint8_t *firmware, size_t size) = 0;
    virtual aditof::Status getFrame(uint16_t *buffer) = 0;
    virtual aditof::Status readEeprom(uint32_t address, uint8_t *data,
                                      size_t length) = 0;
    virtual aditof::Status writeEeprom(uint32_t address, const uint8_t *data,
                                       size_t length) = 0;
    virtual aditof::Status readAfeRegisters(const uint16_t *address,
                                            uint16_t *data, size_t length) = 0;
    virtual aditof::Status writeAfeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length) = 0;
    virtual aditof::Status readAfeTemp(float &temperature) = 0;
    virtual aditof::Status readLaserTemp(float &temperature) = 0;
};

#endif // DEVICE_INTERFACE_H
