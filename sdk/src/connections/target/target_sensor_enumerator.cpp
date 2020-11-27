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
#include "connections/target/target_sensor_enumerator.h"
#include "connections/target/addi9036_sensor.h"
#include "connections/target/chicony_temperature_sensor.h"
#include "connections/target/eeprom.h"
#include "connections/target/i2c_temperature_sensor.h"
#include "target_definitions.h"

#include <sys/stat.h>

static const char *LASER_TEMP_SENSOR_NAME = "LaserTemperature";
static const char *AFE_TEMP_SENSOR_NAME = "AfeTemperature";
static const char *CHICONY_TEMPERATURE_SENSOR = "Chicony Temperature Sensor";

static const char LASER_TEMP_SENSOR_I2C_ADDR = 0x49;
static const char AFE_TEMP_SENSOR_I2C_ADDR = 0x4b;

using namespace aditof;

Status TargetSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        switch (sInfo.sensorType) {
        case SensorType::SENSOR_ADDI9036: {
            auto sensor = std::make_shared<Addi9036Sensor>(sInfo.driverPath,
                                                           sInfo.subDevPath);
            depthSensors.emplace_back(sensor);
            break;
        }
        } //switch (sInfo.sensorType) {
    }

    return Status::OK;
}

Status TargetSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> &storages) {

    storages.clear();

    for (const auto &eInfo : m_storagesInfo) {
        auto storage =
            std::make_shared<Eeprom>(eInfo.driverName, eInfo.driverPath);
        storages.emplace_back(storage);
    }

    return Status::OK;
}

Status TargetSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        &temperatureSensors) {

    temperatureSensors.clear();

    // Check if laser temperature is available
    struct stat st;
    if (stat(TEMP_SENSOR_DEV_PATH, &st) == 0) {
        // TO DO: Find a way to check if device is available at a given I2C address. Now we're just assuming they are available
        auto laserSensor = std::make_shared<I2CTemperatureSensor>(
            LASER_TEMP_SENSOR_NAME, TEMP_SENSOR_DEV_PATH,
            LASER_TEMP_SENSOR_I2C_ADDR);
        temperatureSensors.emplace_back(laserSensor);

        auto afeSensor = std::make_shared<I2CTemperatureSensor>(
            AFE_TEMP_SENSOR_NAME, TEMP_SENSOR_DEV_PATH,
            AFE_TEMP_SENSOR_I2C_ADDR);
        temperatureSensors.emplace_back(afeSensor);
    }

#ifdef CHICONY_006
    auto sensor = std::make_shared<ChiconyTemperatureSensor>(
        CHICONY_TEMPERATURE_SENSOR, TEMP_SENSOR_DEV_PATH);
    temperatureSensors.emplace_back(sensor);
#endif

    return Status::OK;
}
