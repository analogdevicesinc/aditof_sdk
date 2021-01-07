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
#ifndef TEMPERATURE_SENSOR_INTERFACE_H
#define TEMPERATURE_SENSOR_INTERFACE_H

#include "aditof/status_definitions.h"
#include "sdk_exports.h"

#include <string>

namespace aditof {

/**
 * @class TemperatureSensorInterface
 * @brief Interface for a temperature sensor
 */
class SDK_API TemperatureSensorInterface {
  public:
    /**
   * @brief Destructor
   */
    virtual ~TemperatureSensorInterface() = default;

    /**
     * @brief Open the communication channel with the temperature sensor.
     * @param handle - A handle to the object through which communication is done
     * @param name - The name of temperature sensor available on sysfs
     * @param driver_path - The temperature sensor driver path on sysfs
     * @return Status
     */
    virtual aditof::Status open(void *handle, const std::string &name,
                                const std::string &driver_path) = 0;

    /**
     * @brief Read data from the temperature sensor
     * @param[out] temperature - This is set with the temperature read from the sensor
     * @return Status
     */
    virtual aditof::Status read(float &temperature) = 0;

    /**
     * @brief Close the communication channel with the EEPROM.
     * @return Status
     */
    virtual aditof::Status close() = 0;

    /**
     * @brief Retrieves the name of the temperature sensor
     * @param[out] name - This gets set with the name of the temperature sensor
     * @return Status
     */
    virtual aditof::Status getName(std::string &name) const = 0;
};

} // namespace aditof

#endif // TEMPERATURE_SENSOR_INTERFACE_H
