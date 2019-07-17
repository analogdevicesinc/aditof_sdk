#ifndef DEVICE_ENUMERATOR_FACTORY_H
#define DEVICE_ENUMERATOR_FACTORY_H

#include "device_enumerator_interface.h"

#include <memory>

class DeviceEnumeratorFactory {
  public:
    static std::unique_ptr<DeviceEnumeratorInterface> buildDeviceEnumerator();
    static std::unique_ptr<DeviceEnumeratorInterface>
    buildDeviceEnumeratorEthernet(const std::string &ip);
};

#endif // DEVICE_ENUMERATOR_FACTORY_H
