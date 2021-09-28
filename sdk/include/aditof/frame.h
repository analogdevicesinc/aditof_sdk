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
#ifndef FRAME_H
#define FRAME_H

#include "frame_definitions.h"
#include "sdk_exports.h"
#include "status_definitions.h"

#include <memory>

class FrameImpl;

namespace aditof {

/**
 * @class Frame
 * @brief Frame of a camera.
 */
class Frame {
  public:
    /**
     * @brief Constructor
     */
    SDK_API Frame();

    /**
     * @brief Destructor
     */
    SDK_API ~Frame();

    /**
     * @brief Copy constructor
     */
    SDK_API Frame(const Frame &op);

    /**
     * @brief Copy assignment
     */
    SDK_API Frame &operator=(const Frame &op);

    /**
     * @brief Move constructor
     */
    SDK_API Frame(Frame &&) noexcept;

    /**
     * @brief Move assignment
     */
    SDK_API Frame &operator=(Frame &&) noexcept;

  public:
    /**
     * @brief Configures the frame with the given details
     * @param details
     * @return Status
     */
    SDK_API Status setDetails(const FrameDetails &details);

    /**
     * @brief Gets the current details of the frame
     * @param[out] details
     * @return Status
     */
    SDK_API Status getDetails(FrameDetails &details) const;

    /**
     * @brief Gets the address where the specified data is being stored
     * @param dataType
     * @param[out] dataPtr
     * @return Status
     */
    SDK_API Status getData(FrameDataType dataType, uint16_t **dataPtr);

  private:
    std::unique_ptr<FrameImpl> m_impl;
};

} // namespace aditof

#endif // FRAME_H
