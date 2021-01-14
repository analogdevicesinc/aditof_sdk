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
#ifndef TARGET_SENSOR_ENUMERATOR_H
#define TARGET_SENSOR_ENUMERATOR_H

#include "aditof/sensor_definitions.h"
#include "aditof/sensor_enumerator_interface.h"

class TargetSensorEnumerator : public aditof::SensorEnumeratorInterface {
  public:
    ~TargetSensorEnumerator() = default;

  public: // implements SensorEnumeratorInterface
    virtual aditof::Status searchSensors() override;
    virtual aditof::Status
    getDepthSensors(std::vector<std::shared_ptr<aditof::DepthSensorInterface>>
                        &depthSensors) override;
    virtual aditof::Status getStorages(
        std::vector<std::shared_ptr<aditof::StorageInterface>> &storages)
        override;
    virtual aditof::Status getTemperatureSensors(
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &temperatureSensors) override;

  private:
    enum class SensorType {
        SENSOR_ADDI9036, //!< ADDI9036 CCD sensor
    };

    enum class TempSensorType {
        SENSOR_ADT7410, //!< ADT7410 temperature sensor
        SENSOR_TMP10X   //!< TMP101 / TMP102 temperature sensor
    };

    struct StorageInfo {
        std::string driverName;
        std::string driverPath;
    };

    struct SensorInfo {
        SensorType sensorType;
        std::string driverPath;
        std::string subDevPath;
        std::string captureDev;
    };

    struct TemperatureSensorInfo {
        TempSensorType sensorType;
        std::string driverPath;
        int i2c_address;
        std::string name;
    };

    std::vector<SensorInfo> m_sensorsInfo;
    std::vector<StorageInfo> m_storagesInfo;
    std::vector<TemperatureSensorInfo> m_temperatureSensorsInfo;
};

#endif // TARGET_SENSOR_ENUMERATOR_H
