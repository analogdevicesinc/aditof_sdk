#ifndef DEVICE_DEFINITIONS_H
#define DEVICE_DEFINITIONS_H

#include <string>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @enum SensorType
 * @brief Provides the types of sensor assosiated with the device
 */
enum class SensorType {
    SENSOR_96TOF1,  //!< 96Tof 1 sensor
    SENSOR_CHICONY, //!< Chicony sensor
};

/**
 * @struct DeviceDetails
 * @brief Provides details about the device
 */
struct DeviceDetails {
    /**
     * @brief The type of sensor
     */
    SensorType sensorType;
};

} // namespace aditof

#endif // DEVICE_DEFINITIONS_H
