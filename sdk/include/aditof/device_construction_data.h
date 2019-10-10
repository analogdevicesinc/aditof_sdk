#ifndef DEVICE_CONSTRUCTION_DATA
#define DEVICE_CONSTRUCTION_DATA

#include <string>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @enum DeviceType
 * @brief Provides the types of sensor assosiated with the device
 */
enum class DeviceType {
    LOCAL,    //!< on the target
    USB,      //!< connects to target via USB
    ETHERNET, //!< connects to target via Ethernet
};

/**
 * @struct DeviceConstructionData
 * @brief Provides data required to construct a device
 */
struct DeviceConstructionData {
    /**
     * @brief The type of the device
     */
    DeviceType deviceType;

    /**
     * @brief The URL associated with the driver used by the device to talk to
     * hardware
     */
    std::string driverPath;

    /**
     * @brief The IP address of the target to which the device connects to (if
     * is a Ethernet device)
     */
    std::string ip;
};

}; // namespace aditof

#endif // DEVICE_CONSTRUCTION_DATA
