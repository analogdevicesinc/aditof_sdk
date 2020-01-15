#ifndef CAMERA_DEFINITIONS_H
#define CAMERA_DEFINITIONS_H

#include "frame_definitions.h"
#include "status_definitions.h"

#include <functional>
#include <memory>
#include <string>

/**
 * @brief Namespace aditof
 */
namespace aditof {

class Frame;

/**
 * @brief Callback for frame updates
 */
typedef std::function<void(Status, Frame *)> FrameUpdateCallback;

/**
 * @enum ConnectionType
 * @brief Provides the types of connection with a Camera
 */
enum class ConnectionType {
    USB,      //!< A USB connnection
    ETHERNET, //!< An ethernet connection
    LOCAL     //!< The device is embedded in the system
};

/**
 * @struct CameraDetails
 * @brief Describes the properties of a camera.
 */
struct CameraDetails {
    /**
     * @brief Camera identification
     */
    std::string cameraId;

    /**
     * @brief The mode in which the camera operates
     */
    std::string mode;

    /**
     * @brief Details about the frames that camera is capturing
     */
    FrameDetails frameType;

    /**
     * @brief The type of connection with the camera
     */
    ConnectionType connection;

    /**
     * @brief The range of the camera in mm for the operating mode
     */
    int range;

    /**
     * @brief The number of bits used for representing one pixel data.
     */
    int bitCount;
};

} // namespace aditof

#endif // CAMERA_DEFINITIONS_H
