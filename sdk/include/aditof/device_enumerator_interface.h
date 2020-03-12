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
#ifndef DEVICE_ENUMERATOR_INTERFACE_H
#define DEVICE_ENUMERATOR_INTERFACE_H

#include <aditof/device_construction_data.h>
#include <aditof/status_definitions.h>

#include <vector>

namespace aditof {

/**
 * @class DeviceEnumeratorInterface
 * @brief Provides the mean to find and access devices.
 */
class DeviceEnumeratorInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~DeviceEnumeratorInterface() = default;

    /**
     * @brief Find devices that are available. A device is available if either
     * has an associated drivers (happens on target) or is connected to via USB
     * to the system (usually on a host).
     * @param[out] devices - list of found devices
     * @return Status
     */
    virtual aditof::Status
    findDevices(std::vector<aditof::DeviceConstructionData> &devices) = 0;
};

} // namespace aditof

#endif // DEVICE_ENUMERATOR_INTERFACE_H
