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
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

using namespace aditof;

aditof::Status findDevicePathsAtMedia(std::string &dev_name,
                                      std::string &subdev_name,
                                      std::string sensorType) {

    char *buf;
    int size = 0;

    /* Checking for available devices */
    char cmd[96];
    if (sensorType == "addi9036")
        sprintf(cmd, "v4l2-ctl --list-devices | grep addi9036 -A 2 | sed '1d' "
                     "| sed 's/^[[:space:]]*//g' | sed '2d'");
    else if (sensorType == "ov2735")
        sprintf(cmd, "v4l2-ctl --list-devices | grep ov2735 -A 2 | sed '1d' | "
                     "sed 's/^[[:space:]]*//g' | sed '2d'");
    FILE *fp = popen(cmd, "r");
    if (!fp) {
        LOG(WARNING) << "Error running v4l2-ctl";
        return Status::GENERIC_ERROR;
    }

    /* Read the media-ctl output stream */
    buf = (char *)malloc(128 * 1024);
    while (!feof(fp)) {
        fread(&buf[size], 1, 1, fp);
        size++;
    }
    pclose(fp);
    std::string str(buf);
    str.resize(size - 2);

    /*Check if the obtained file has content dev and vide in it*/
    if (str.find("dev") == std::string::npos ||
        str.find("video") == std::string::npos) {
        LOG(WARNING) << "Generic error";
        return Status::GENERIC_ERROR;
    }

    dev_name = str;
    subdev_name = str;
    return Status::OK;
}

Status TargetSensorEnumerator::searchSensors() {
    Status status = Status::OK;
    LOG(INFO) << "Looking for devices on the target: Jetson";

    //Depth camera
    SensorInfo sInfo;
    sInfo.sensorType = SensorType::SENSOR_ADDI9036;

    std::string devPath;
    std::string subdevPath;

    status = findDevicePathsAtMedia(devPath, subdevPath, "addi9036");
    if (status != Status::OK) {
        LOG(WARNING) << "failed to find device paths";
        return status;
    }

    if (devPath.empty() || subdevPath.empty()) {
        return Status::GENERIC_ERROR;
    }

    sInfo.driverPath = devPath;
    sInfo.subDevPath = subdevPath;
    sInfo.captureDev = DEPTH_CAPTURE_DEVICE_NAME;
    m_sensorsInfo.emplace_back(sInfo);

    //RGB Camera
    sInfo.sensorType = SensorType::SENSOR_OV2735;

    status = findDevicePathsAtMedia(devPath, subdevPath, "ov2735");
    if (status != Status::OK) {
        LOG(WARNING) << "failed to find device paths";
        return status;
    }

    if (devPath.empty() || subdevPath.empty()) {
        return Status::GENERIC_ERROR;
    }

    sInfo.driverPath = devPath;
    sInfo.subDevPath = subdevPath;
    sInfo.captureDev = RGB_CAPTURE_DEVICE_NAME;
    m_sensorsInfo.emplace_back(sInfo);

    StorageInfo eepromInfo;
    eepromInfo.driverName = EEPROM_NAME;
    eepromInfo.driverPath = EEPROM_DEV_PATH;
    m_storagesInfo.emplace_back(eepromInfo);

    TemperatureSensorInfo temperatureSensorsInfo;
    temperatureSensorsInfo.sensorType = TempSensorType::SENSOR_TMP10X;
    temperatureSensorsInfo.driverPath = TEMP_SENSOR_DEV_PATH;
    temperatureSensorsInfo.name = TEMPERATURE_SENSOR_NAME;
    m_temperatureSensorsInfo.emplace_back(temperatureSensorsInfo);

    return Status::OK;
}
