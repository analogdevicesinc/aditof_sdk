#ifndef DEVICE_FACTORY_H
#define DEVICE_FACTORY_H

#include <aditof/device_construction_data.h>
#include <aditof/device_interface.h>

namespace aditof {

/**
 * @class DeviceFactory
 * @brief Provides the means to construct different types of devices
 */
class DeviceFactory {
  public:
    /**
     * @brief Factory method to create a device.
     * @return DeviceInterface*
     */
    static DeviceInterface *
    buildDevice(const aditof::DeviceConstructionData &data);
};

} // namespace aditof

#endif // DEVICE_FACTORY_H
