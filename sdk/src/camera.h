#ifndef CAMERA_H
#define CAMERA_H

#include "camera_definitions.h"
#include "sdk_exports.h"
#include "status_definitions.h"

#include <functional>
#include <memory>
#include <vector>

class CameraImpl;
class DeviceInterface;

namespace aditof {

class Frame;

/**
 * @class Camera
 * @brief Manipulates the underlying camera system
 */
class SDK_API Camera {
  public:
    /**
     * @brief Constructor
     */
    Camera(std::unique_ptr<CameraImpl> impl);

    /**
     * @brief Destructor
     */
    ~Camera();

    // Make Camera movable and non-copyable
    /**
     * @brief Move constructor
     */
    Camera(Camera &&op) noexcept;

    /**
     * @brief Move assignment
     */
    Camera &operator=(Camera &&op) noexcept;

  public:
    /**
     * @brief Initialize the camera. This is required before performing any
     * operation on the camera.
     * @return Status
     */
    Status initialize();

    /**
     * @brief Start the camera. This starts the streaming of data from the
     * camera.
     * @return Status
     */
    Status start();

    /**
     * @brief Stop the camera. This makes the camera to stop streaming.
     * @return Status
     */
    Status stop();

    /**
     * @brief Puts the camera into the given mode.
     * @param mode - The mode of the camera
     * @param modeFilename - The configuration file which should be specific to
     * the given mode. If no file is specified, the default one for the given
     * mode will be used.
     * @return Status
     */
    Status setMode(const std::string &mode,
                   const std::string &modeFilename = {});

    /**
     * @brief Returns all the modes that are supported by the camera
     * @param[out] availableModes
     * @return Status
     */
    Status getAvailableModes(std::vector<std::string> &availableModes) const;

    /**
     * @brief Set the camera frame type to the givn type
     * @param frameType - The frame type of the camera
     * @return Status
     */
    Status setFrameType(const std::string &frameType);

    /**
     * @brief Returns all the frame types that are supported by the camera
     * @param[out] availableFrameTypes
     * @return Status
     */
    Status
    getAvailableFrameTypes(std::vector<std::string> &availableFrameTypes) const;

    /**
     * @brief Captures data from the camera and assigns it to the given frame.
     * If cb parameter is not given this operation will be blocking. If a
     * callback is provided this operation will be unblocking and once the data
     * for the frame is ready, an internal thread will call the specified
     * callback.
     * @param frame - The frame to which the camera data should be assign
     * @param cb - Callback to be called when frame is updated
     * @return Status
     */
    Status requestFrame(std::shared_ptr<Frame> frame,
                        FrameUpdateCallback cb = nullptr);

    /**
     * @brief Gets the current details of the camera
     * @param[out] details
     * @return Status
     */
    Status getDetails(CameraDetails &details) const;

    /**
     * @brief Gets the device of the camera
     * @return DeviceInterface
     */
    std::shared_ptr<DeviceInterface> getDevice();

  private:
    std::unique_ptr<CameraImpl> m_impl;
};

} // namespace aditof

#endif // CAMERA_H
