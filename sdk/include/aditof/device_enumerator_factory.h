#ifndef DEVICE_ENUMERATOR_FACTORY_H
#define DEVICE_ENUMERATOR_FACTORY_H

#include "aditof/device_enumerator_interface.h"

#include <memory>

/**
 * @class DeviceEnumeratorFactory
 * @brief Provides the means to construct different types of device enumerators
 */
class DeviceEnumeratorFactory {
  public:
    /**
     * @brief Factory method to create a device enumerator on the system.
     * @return std::unique_ptr<DeviceEnumeratorInterface>
     */
    static std::unique_ptr<DeviceEnumeratorInterface> buildDeviceEnumerator();

    /**
     * @brief Factory method to create a device enumerator over ethernet.
     * @return std::unique_ptr<DeviceEnumeratorInterface>
     */
    static std::unique_ptr<DeviceEnumeratorInterface>
    buildDeviceEnumeratorEthernet(const std::string &ip);
};

#endif // DEVICE_ENUMERATOR_FACTORY_H
