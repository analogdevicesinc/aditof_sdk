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
#ifndef SYSTEM_H
#define SYSTEM_H

#include "sdk_exports.h"
#include "status_definitions.h"

#include <memory>
#include <string>
#include <vector>

class SystemImpl;

namespace aditof {

class Camera;

/**
 * @class System
 * @brief The TOF system that manages the cameras.
 */
class SDK_API System {
  public:
    /**
     * @brief Constructor
     */
    System();

    /**
     * @brief Destructor
     */
    ~System();

    // Make System movable and non-copyable
    /**
     * @brief Move constructor
     */
    System(System &&op) noexcept;

    /**
     * @brief Move assignment
     */
    System &operator=(System &&op) noexcept;

  public:
    /**
     * @brief Populates the given list with Camera objects that correspond to
     * the available cameras.
     * @param[out] cameraList - A container to be set with the available cameras
     * @return Status
     */
    Status
    getCameraList(std::vector<std::shared_ptr<Camera>> &cameraList) const;

    /**
     * @brief Populates the given list with Camera objects that correspond to
     * the available cameras from the remote target.
     * @param[out] cameraList - A container to be set with the available cameras
     * @param ip - The IP of the remote target
     * @return Status
     */
    Status getCameraListAtIp(std::vector<std::shared_ptr<Camera>> &cameraList,
                             const std::string &ip) const;

  private:
    std::unique_ptr<SystemImpl> m_impl;
};

} // namespace aditof

#endif // SYSTEM_H
