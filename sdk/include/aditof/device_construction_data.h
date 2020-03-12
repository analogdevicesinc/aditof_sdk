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
