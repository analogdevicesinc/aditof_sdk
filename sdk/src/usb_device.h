#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include "device_construction_data.h"
#include <aditof/device_interface.h>

#include <memory>

class UsbDevice : public DeviceInterface {
  public:
    UsbDevice(const aditof::DeviceConstructionData &data);
    ~UsbDevice();

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
    virtual aditof::Status setCalibrationParams(const std::string &mode,
                                                float gain, float offset);
    virtual aditof::Status applyCalibrationToFrame(uint16_t *frame,
                                                   const std::string &mode);

  private:
    struct ImplData;

    aditof::DeviceConstructionData m_devData;
    std::unique_ptr<ImplData> m_implData;
};

#endif // USB_DEVICE_H
