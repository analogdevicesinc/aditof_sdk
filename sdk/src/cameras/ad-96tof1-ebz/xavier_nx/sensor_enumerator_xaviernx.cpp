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

#include <bits/stdc++.h>
#include <dirent.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <regex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

using namespace aditof;

Status findDevicePathsAtMedia(std::string &dev_name, std::string &subdev_name) {
    using namespace aditof;
    using namespace std;

    //DevPaths
    char *buf;
    int size = 0;
    /* Checking for available devices */
    char cmdDev[96];
    sprintf(cmdDev, "v4l2-ctl --list-devices | grep addi9036 -A 2 | sed '1d'");
    FILE *fp = popen(cmdDev, "r");
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
    std::string strDev(buf);
    free(buf);
    std::regex e{R"(\/dev\/video\d)"};
    std::sregex_iterator devIter(strDev.begin(), strDev.end(), e);
    std::sregex_iterator end;
    int nrOfDevPaths = 0;
    while (devIter != end) {
        for (unsigned i = 0; i < devIter->size(); ++i) {
            if (nrOfDevPaths == 0)
                dev_name.append((*devIter)[i]);
            else {
                dev_name.append(std::string("|"));
                dev_name.append((*devIter)[i]);
            }
        }
        ++devIter;
        nrOfDevPaths++;
    }

    //SubDevPaths
    char *subDevBuf;
    int subDevSize = 0;
    /* Checking for available devices */
    char cmdSubDev[2500];
    sprintf(cmdSubDev, "media-ctl -p");
    FILE *subDevFp = popen(cmdSubDev, "r");
    if (!subDevFp) {
        LOG(WARNING) << "Error running media-ctl";
        return Status::GENERIC_ERROR;
    }

    /* Read the media-ctl output stream */
    subDevBuf = (char *)malloc(2500 * sizeof(char));
    while (!feof(subDevFp)) {
        fread(&subDevBuf[subDevSize], 1, 1, subDevFp);
        subDevSize++;
    }
    pclose(subDevFp);
    std::string strSubDev(subDevBuf);
    free(subDevBuf);
    std::regex subE{R"(- entity [\d]+: addi9036[a-zA-Z0-9_.+\-([,)\n /]+)"};
    std::sregex_iterator subDevIter(strSubDev.begin(), strSubDev.end(), subE);
    int nrOfSubDevPaths = 0;
    while (subDevIter != end) {
        for (unsigned i = 0; i < subDevIter->size(); ++i) {
            std::string tmp = (*subDevIter)[i];
            std::regex subsubE{R"(/dev/v[0-9]+l-subdev[0-9]+\b)"};
            std::sregex_iterator subSubDevIter(tmp.begin(), tmp.end(), subsubE);
            while (subSubDevIter != end) {
                for (unsigned j = 0; j < subSubDevIter->size(); ++j) {
                    if (nrOfSubDevPaths >= 1)
                        subdev_name.append("|");
                    subdev_name.append((*subSubDevIter)[j]);
                }
                ++subSubDevIter;
            }
        }
        ++subDevIter;
        nrOfSubDevPaths++;
    }
    return Status::OK;
}

Status TargetSensorEnumerator::searchSensors() {

    Status status = Status::OK;
    LOG(INFO) << "Looking for devices on the target: Xavier NX";

    SensorInfo sInfo;
    sInfo.sensorType = SensorType::SENSOR_ADDI9036;
    std::string devPath;
    std::string subdevPath;

    status = findDevicePathsAtMedia(devPath, subdevPath);
    if (status != Status::OK) {
        LOG(WARNING) << "failed to find device paths";
        return status;
    }

    if (devPath.empty() || subdevPath.empty()) {
        return Status::GENERIC_ERROR;
    }

    sInfo.driverPath = devPath;
    sInfo.subDevPath = subdevPath;
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
