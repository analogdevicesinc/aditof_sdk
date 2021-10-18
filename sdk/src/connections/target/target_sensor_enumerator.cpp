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
#include "connections/target/adt7410_sensor.h"
#include "connections/target/eeprom.h"
#include "connections/target/rgb_sensor.h"
#include "connections/target/rgbd_sensor.h"
#include "connections/target/tmp10x_sensor.h"
#include <algorithm>

using namespace aditof;
using namespace std;
Status TargetSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {
    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        switch (sInfo.sensorType) {
        case SensorType::SENSOR_ADDI9036: {
            auto sensor = std::make_shared<Addi9036Sensor>(
                sInfo.driverPath, sInfo.subDevPath, sInfo.captureDev);
            depthSensors.emplace_back(sensor);
            break;
        }
        case SensorType::SENSOR_OV2735: {
            auto sensor = std::make_shared<RgbSensor>(
                sInfo.driverPath, sInfo.subDevPath, sInfo.captureDev);
            depthSensors.emplace_back(sensor);
            break;
        }
        }
    }

    std::vector<std::shared_ptr<DepthSensorInterface>>::iterator
        tmpDepthSensor = std::find_if(depthSensors.begin(), depthSensors.end(),
                                      [](shared_ptr<DepthSensorInterface> aux) {
                                          std::string tmpName;
                                          aux.get()->getName(tmpName);
                                          return (tmpName == "addi9036");
                                      });
    std::vector<std::shared_ptr<DepthSensorInterface>>::iterator tmpRgbSensor =
        std::find_if(depthSensors.begin(), depthSensors.end(),
                     [](shared_ptr<DepthSensorInterface> aux) {
                         std::string tmpName;
                         aux.get()->getName(tmpName);
                         return (tmpName == "ov2735");
                     });

    if (tmpDepthSensor != depthSensors.end() &&
        tmpRgbSensor != depthSensors.end()) {
        auto sensor =
            std::make_shared<RgbdSensor>(*tmpDepthSensor, *tmpRgbSensor);
        depthSensors.erase(tmpDepthSensor);
        depthSensors.erase(tmpRgbSensor);
        depthSensors.emplace_back(sensor);
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

    for (const auto &tInfo : m_temperatureSensorsInfo) {
        switch (tInfo.sensorType) {
        case TempSensorType::SENSOR_ADT7410: {
            auto sensor = std::make_shared<ADT7410>(
                tInfo.name, tInfo.driverPath, tInfo.i2c_address);
            temperatureSensors.emplace_back(sensor);
            break;
        }
        case TempSensorType::SENSOR_TMP10X: {
            auto sensor =
                std::make_shared<TMP10x>(tInfo.name, tInfo.driverPath);
            temperatureSensors.emplace_back(sensor);
            break;
        }
        }
    }

    return Status::OK;
}

Status TargetSensorEnumerator::getCameraTypeOnTarget(CameraType &cameraType) {
#if defined(FXTOF1)
    cameraType = CameraType::AD_FXTOF1_EBZ;
#elif defined(SMART_3D)
    cameraType = CameraType::SMART_3D_CAMERA;
#else
    cameraType = CameraType::AD_96TOF1_EBZ;
#endif

    return Status::OK;
}
