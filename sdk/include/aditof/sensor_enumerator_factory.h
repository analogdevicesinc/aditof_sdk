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
#ifndef SENSOR_ENUMERATOR_FACTORY_H
#define SENSOR_ENUMERATOR_FACTORY_H

#include "aditof/sensor_enumerator_interface.h"
#include "sdk_exports.h"

#include <memory>

namespace aditof {

/**
 * @class SensorEnumeratorFactory
 * @brief Provides the means to construct different types of sensors enumerators.
 * Based on the connection type (on target, USB, Network), different enumerators need
 * to be used.
 */
class SDK_API SensorEnumeratorFactory {
  public:
    /**
     * @brief Factory method to create an enumerator to look for sensors on target.
     * Factory method will return null if the call is not made on target.
     * @return std::unique_ptr<SensorEnumeratorInterface>
     */
    static std::unique_ptr<SensorEnumeratorInterface>
    buildTargetSensorEnumerator();

    /**
     * Factory method to create an enumerator to look for sensors over USB.
     * @return std::unique_ptr<DeviceEnumeratorInterface>
     */
    static std::unique_ptr<SensorEnumeratorInterface>
    buildUsbSensorEnumerator();

    /**
     * Factory method to create an enumerator to look for sensors over network.
     * @return std::unique_ptr<DeviceEnumeratorInterface>
     */
    static std::unique_ptr<SensorEnumeratorInterface>
    buildNetworkSensorEnumerator(const std::string &ip);
};

} // namespace aditof

#endif // SENSOR_ENUMERATOR_FACTORY_H
