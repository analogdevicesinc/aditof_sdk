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
class SDK_API Frame {
  public:
    /**
     * @brief Constructor
     */
    Frame();

    /**
     * @brief Destructor
     */
    ~Frame();

    /**
     * @brief Copy constructor
     */
    Frame(const Frame &op);

    /**
     * @brief Copy assignment
     */
    Frame &operator=(const Frame &op);

    /**
     * @brief Move constructor
     */
    Frame(Frame &&) noexcept;
    /**
     * @brief Move assignment
     */
    Frame &operator=(Frame &&) noexcept;

  public:
    /**
     * @brief Configures the frame with the given details
     * @param details
     * @return Status
     */
    Status setDetails(const FrameDetails &details);

    /**
     * @brief Gets the current details of the frame
     * @param[out] details
     * @return Status
     */
    Status getDetails(FrameDetails &details) const;

    /**
     * @brief Gets details of a type of data within the frame
     * @param dataType
     * @param[out] details
     * @return Status
     */
    Status getDataDetails(const std::string &dataType,
                          FrameDataDetails &details) const;

    /**
     * @brief Gets the address where the specified data is being stored
     * @param dataType
     * @param[out] dataPtr
     * @return Status
     */
    Status getData(const std::string &dataType, uint16_t **dataPtr);

    /**
     * @brief Gets the frames's list of attributes
     * @param[out] attributes
     * @return Status
     */
    virtual Status
    getAvailableAttributes(std::vector<std::string> &attributes) const = 0;

    /**
     * @brief Sets a specific frame attribute
     * @param[in] attribute - Attribute name
     * @param[in] value - Attribute value
     * @return Status
     */
    virtual Status setAttribute(const std::string &attribute,
                                const std::string &value) = 0;

    /**
     * @brief Gets the value of a specific frame attribute
     * @param[in] attribute - Attribute name
     * @param[out] value - Attribute value
     * @return Status
     */
    virtual Status getAttribute(const std::string &attribute,
                                std::string &value) const = 0;

  private:
    std::unique_ptr<FrameImpl> m_impl;
};

} // namespace aditof

#endif // FRAME_H
