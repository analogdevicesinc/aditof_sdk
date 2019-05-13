#ifndef ETHERNET_DEVICE_H
#define ETHERNET_DEVICE_H

#include "device_construction_data.h"
#include "device_interface.h"

class EthernetDevice : public DeviceInterface {
  public:
    EthernetDevice(const aditof::DeviceConstructionData &data);
    ~EthernetDevice() = default;

  public: // implements DeviceInterface
    virtual aditof::Status open();
    virtual aditof::Status start();
    virtual aditof::Status stop();
    virtual aditof::Status
    getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types);
    virtual aditof::Status setFrameType(const aditof::FrameDetails &details);
    virtual aditof::Status program(const uint8_t *firmware, size_t size);
    virtual aditof::Status getFrame(uint16_t *buffer);
    virtual aditof::Status readEeprom(uint32_t address, uint8_t *data,
                                      size_t length);
    virtual aditof::Status writeEeprom(uint32_t address, const uint8_t *data,
                                       size_t length);
    virtual aditof::Status readAfeRegisters(const uint16_t *address,
                                            uint16_t *data, size_t length);
    virtual aditof::Status writeAfeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length);
    virtual aditof::Status readAfeTemp(float &temperature);
    virtual aditof::Status readLaserTemp(float &temperature);
};

#endif // ETHERNET_DEVICE_H
