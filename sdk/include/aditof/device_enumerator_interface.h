#ifndef DEVICE_ENUMERATOR_INTERFACE_H
#define DEVICE_ENUMERATOR_INTERFACE_H

#include <aditof/device_construction_data.h>
#include <aditof/status_definitions.h>

#include <vector>

/**
 * @class DeviceEnumeratorInterface
 * @brief Provides the mean to find and access devices.
 */
class DeviceEnumeratorInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~DeviceEnumeratorInterface() = default;

    /**
     * @brief Find devices that are available. A device is available if either
     * has an associated drivers (happens on target) or is connected to via USB
     * to the system (usually on a host).
     * @param[out] devices - list of found devices
     * @return Status
     */
    virtual aditof::Status
    findDevices(std::vector<aditof::DeviceConstructionData> &devices) = 0;
};

#endif // DEVICE_ENUMERATOR_INTERFACE_H
