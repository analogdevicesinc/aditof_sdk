#ifndef DEVICE_FACTORY_H
#define DEVICE_FACTORY_H

#include "device_construction_data.h"
#include "device_interface.h"

#include <memory>

class DeviceFactory {
  public:
    static std::unique_ptr<DeviceInterface>
    buildDevice(const aditof::DeviceConstructionData &data);
};

#endif // DEVICE_FACTORY_H
