/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef CAMERA_H
#define CAMERA_H

#include "camera_definitions.h"
#include "sdk_exports.h"
#include "status_definitions.h"

#include <functional>
#include <string>
#include <vector>

namespace aditof {

class Frame;
class CameraSpecifics;
class DeviceInterface;

/**
 * @class Camera
 * @brief Manipulates the underlying camera system
 */
class SDK_API Camera {
  public:
    /**
     * @brief Destructor
     */
    virtual ~Camera() = default;

    /**
     * @brief Initialize the camera. This is required before performing any
     * operation on the camera.
     * @return Status
     */
    virtual Status initialize() = 0;

    /**
     * @brief Start the camera. This starts the streaming of data from the
     * camera.
     * @return Status
     */
    virtual Status start() = 0;

    /**
     * @brief Stop the camera. This makes the camera to stop streaming.
     * @return Status
     */
    virtual Status stop() = 0;

    /**
     * @brief Puts the camera into the given mode.
     * @param mode - The mode of the camera
     * @param modeFilename - When there is a need to use a custom mode
     * then mode parameter needs to be set to 'custom' and a firmware
     * file needs to be provided.
     * @return Status
     */
    virtual Status setMode(const std::string &mode,
                           const std::string &modeFilename = {}) = 0;

    /**
     * @brief Returns all the modes that are supported by the camera
     * @param[out] availableModes
     * @return Status
     */
    virtual Status
    getAvailableModes(std::vector<std::string> &availableModes) const = 0;

    /**
     * @brief Set the camera frame type to the given type
     * @param frameType - The frame type of the camera
     * @return Status
     */
    virtual Status setFrameType(const std::string &frameType) = 0;

    /**
     * @brief Returns all the frame types that are supported by the camera
     * @param[out] availableFrameTypes
     * @return Status
     */
    virtual Status getAvailableFrameTypes(
        std::vector<std::string> &availableFrameTypes) const = 0;

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
    virtual Status requestFrame(Frame *frame,
                                FrameUpdateCallback cb = nullptr) = 0;

    /**
     * @brief Gets the current details of the camera
     * @param[out] details
     * @return Status
     */
    virtual Status getDetails(CameraDetails &details) const = 0;

    /**
     * @brief Gets an object which extends the API of this camera with
     * additional methods that are specific to this camera. The returned
     * object must be cast to an actual implementation to be able to access
     * the extended API.
     * @return std::shared_ptr<CameraSpecifics>
     */
    virtual std::shared_ptr<CameraSpecifics> getSpecifics() = 0;

    /**
     * @brief Gets the device of the camera. The device is ownen by the camera,
     * therefore when the camera gets destroy the reference to the device will
     * not be valid anymore.
     * @return std::shared_ptr<DeviceInterface>
     */
    virtual std::shared_ptr<DeviceInterface> getDevice() = 0;
};

} // namespace aditof

#endif // CAMERA_H
