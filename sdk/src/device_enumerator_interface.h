#ifndef DEVICE_ENUMERATOR_INTERFACE_H
#define DEVICE_ENUMERATOR_INTERFACE_H

#include "aditof/device_construction_data.h"
#include "aditof/status_definitions.h"

#include <vector>

class DeviceEnumeratorInterface {
  public:
    virtual ~DeviceEnumeratorInterface() = default;

    virtual aditof::Status
    findDevices(std::vector<aditof::DeviceConstructionData> &devices) = 0;
};

#endif // DEVICE_ENUMERATOR_INTERFACE_H
