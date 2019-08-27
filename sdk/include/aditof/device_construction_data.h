#ifndef DEVICE_CONSTRUCTION_DATA
#define DEVICE_CONSTRUCTION_DATA

#include <string>

namespace aditof {

enum class DeviceType { LOCAL, USB, ETHERNET };

struct DeviceConstructionData {
    DeviceType deviceType;
    std::string driverPath;
    std::string ip;
};

}; // namespace aditof

#endif // DEVICE_CONSTRUCTION_DATA
