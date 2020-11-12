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
#ifndef SENSOR_ENUMERATOR_INTERFACE_H
#define SENSOR_ENUMERATOR_INTERFACE_H

#include "aditof/depth_sensor_interface.h"
#include "aditof/status_definitions.h"
#include "aditof/storage_interface.h"
#include "aditof/temperature_sensor_interface.h"

#include <memory>
#include <vector>

namespace aditof {

/**
 * @class SensorEnumeratorInterface
 * @brief Can search for sensors and retrieve sensors by category.
 */
class SensorEnumeratorInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~SensorEnumeratorInterface() = default;

    /**
     * @brief Do a search for the available sensors.
     * @return Status
     */
    virtual aditof::Status searchSensors() = 0;

    /**
     * @brief Get the available depth sensors.
     * @param[out] depthSensors - list of found sensors
     * @return Status
     */
    virtual aditof::Status
    getDepthSensors(std::vector<std::shared_ptr<aditof::DepthSensorInterface>>
                        &depthSensors) = 0;

    /**
     * @brief Get the available storage.
     * @param[out] storages - list of found storages
     * @return Status
     */
    virtual aditof::Status getStorages(
        std::vector<std::shared_ptr<aditof::StorageInterface>> &storages) = 0;

    /**
     * @brief Get the available temperature sensors.
     * @param[out] temperatureSensors - list of found sensors
     * @return Status
     */
    virtual aditof::Status getTemperatureSensors(
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &temperatureSensors) = 0;
};

} // namespace aditof

#endif // SENSOR_ENUMERATOR_INTERFACE_H
