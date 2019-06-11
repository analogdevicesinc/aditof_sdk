#ifndef DEVICE_FACTORY_H
#define DEVICE_FACTORY_H

#include "device_construction_data.h"
#include <aditof/device_interface.h>

class DeviceFactory {
  public:
    static DeviceInterface *
    buildDevice(const aditof::DeviceConstructionData &data);
};

#endif // DEVICE_FACTORY_H
