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
#include "sensor_names.h"
#include "target_definitions.h"

#include <dirent.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

using namespace aditof;

Status TargetSensorEnumerator::searchSensors() {
    LOG(INFO) << "Looking for devices on the target: Jetson";

    // TO DO: Don't guess the device, find a way to identify it so we are sure
    // we've got the right sensor and it's compatible with the SDK
    SensorInfo sInfo;
    sInfo.sensorType = SensorType::SENSOR_ADDI9036;
    sInfo.driverPath = "/dev/video0";
    sInfo.subDevPath = "/dev/video0";
    sInfo.captureDev = CAPTURE_DEVICE_NAME;
    m_sensorsInfo.emplace_back(sInfo);

    StorageInfo eepromInfo;
    eepromInfo.driverName = EEPROM_NAME;
    eepromInfo.driverPath = EEPROM_DEV_PATH;
    m_storagesInfo.emplace_back(eepromInfo);

    TemperatureSensorInfo temperatureSensorsInfo;
    temperatureSensorsInfo.sensorType = TempSensorType::SENSOR_ADT7410;
    temperatureSensorsInfo.driverPath = TEMP_SENSOR_DEV_PATH;
    temperatureSensorsInfo.i2c_address = LASER_TEMP_SENSOR_I2C_ADDR;
    temperatureSensorsInfo.name = LASER_TEMPERATURE_SENSOR_NAME;
    m_temperatureSensorsInfo.emplace_back(temperatureSensorsInfo);

    temperatureSensorsInfo.sensorType = TempSensorType::SENSOR_ADT7410;
    temperatureSensorsInfo.driverPath = TEMP_SENSOR_DEV_PATH;
    temperatureSensorsInfo.i2c_address = AFE_TEMP_SENSOR_I2C_ADDR;
    temperatureSensorsInfo.name = AFE_TEMPERATURE_SENSOR_NAME;

    m_temperatureSensorsInfo.emplace_back(temperatureSensorsInfo);

    return Status::OK;
}
