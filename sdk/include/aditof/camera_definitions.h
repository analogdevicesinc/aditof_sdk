#ifndef CAMERA_DEFINITIONS_H
#define CAMERA_DEFINITIONS_H

#include "frame_definitions.h"
#include "status_definitions.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

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
 * @brief Describes the types of connection with a Camera
 */
enum class ConnectionType {
    USB,      //!< A USB connnection
    ETHERNET, //!< An ethernet connection
    LOCAL     //!< The device is embedded in the system
};

/**
 * @struct IntrinsicParameters
 * @brief Describes the intrinsic parameters of a camera.
 */
struct IntrinsicParameters {
    /**
     * @brief The 3x3 intrinsic parameter matrix (a.k.a K matrix) with values
     * specified in pixel units.
     */
    std::vector<float> cameraMatrix;

    /**
     * @brief The distortion coefficients
     */
    std::vector<float> distCoeffs;

    /**
     * @brief The width of a sensor unit cell specified in mm.
     */
    float pixelWidth;

    /**
     * @brief The height of a sensor unit cell specified in mm.
     */
    float pixelHeight;
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
     * @brief Details about the intrinsic parameters of the camera
     */
    IntrinsicParameters intrinsics;

    /**
     * @brief The maximum distance (in millimeters) the camera can measure in
     * the current operating mode.
     */
    int maxDepth;

    /**
     * @brief The minimum distance (in millimeters) the camera can measure in
     * the current operating mode.
     */
    int minDepth;

    /**
     * @brief The number of bits used for representing one pixel data.
     */
    int bitCount;
};

} // namespace aditof

#endif // CAMERA_DEFINITIONS_H
