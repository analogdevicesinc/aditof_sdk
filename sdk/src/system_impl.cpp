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
#include "system_impl.h"
#include "aditof/sensor_enumerator_factory.h"
#include "aditof/sensor_enumerator_interface.h"
#include "camera_96tof1.h"
#include "camera_chicony_006.h"
#include "camera_fxtof1.h"

#include <aditof/camera.h>
#include <algorithm>
#include <glog/logging.h>

using namespace aditof;

static const std::string skEeprom_24c1024 = "24c1024";
static const std::string skAfeTempSensor = "AfeTemperature";
static const std::string skLaserTempSensor = "LaserTemperature";
static const std::string skChiconyTempSensor = "Chicony Temperature Sensor";

static std::vector<std::shared_ptr<Camera>>
buildCameras(std::unique_ptr<SensorEnumeratorInterface> enumerator) {

    std::vector<std::shared_ptr<Camera>> cameras;
    std::vector<std::shared_ptr<DepthSensorInterface>> depthSensors;
    std::vector<std::shared_ptr<StorageInterface>> storages;
    std::vector<std::shared_ptr<TemperatureSensorInterface>> temperatureSensors;

    enumerator->getDepthSensors(depthSensors);
    enumerator->getStorages(storages);
    enumerator->getTemperatureSensors(temperatureSensors);

    for (const auto &dSensor : depthSensors) {
        SensorDetails dSensorDetails;
        dSensor->getDetails(dSensorDetails);

        switch (dSensorDetails.sensorType) {
        case SensorType::SENSOR_ADDI9036: {
            // Look for EEPROM
            auto eeprom_iter =
                std::find_if(storages.begin(), storages.end(),
                             [](std::shared_ptr<StorageInterface> storage) {
                                 std::string name;
                                 storage->getName(name);
                                 return name == skEeprom_24c1024;
                             });
            if (eeprom_iter == storages.end()) {
                DLOG(INFO)
                    << "Could not find " << skEeprom_24c1024
                    << " while looking for storage for camera AD-96TOF1-EBZ";
                break;
            }

            // Look for AFE temperature sensor
            auto afeTempSensorIter = std::find_if(
                temperatureSensors.begin(), temperatureSensors.end(),
                [](std::shared_ptr<TemperatureSensorInterface> tSensor) {
                    std::string name;
                    tSensor->getName(name);
                    return name == skAfeTempSensor;
                });
            if (afeTempSensorIter == temperatureSensors.end()) {
                DLOG(INFO) << "Could not find " << skAfeTempSensor
                           << " while looking for temperature sensors for "
                              "camera AD-96TOF1-EBZ";
                break;
            }

            // Look for laser temperature sensor
            auto laserTempSensorIter = std::find_if(
                temperatureSensors.begin(), temperatureSensors.end(),
                [](std::shared_ptr<TemperatureSensorInterface> tSensor) {
                    std::string name;
                    tSensor->getName(name);
                    return name == skLaserTempSensor;
                });
            if (laserTempSensorIter == temperatureSensors.end()) {
                DLOG(INFO) << "Could not find " << skLaserTempSensor
                           << " while looking for temperature sensors for "
                              "camera AD-96TOF1-EBZ";
                break;
            }

            std::shared_ptr<StorageInterface> eeprom = *eeprom_iter;
            std::shared_ptr<TemperatureSensorInterface> afeTempSensor =
                *afeTempSensorIter;
            std::shared_ptr<TemperatureSensorInterface> laserTempSensor =
                *laserTempSensorIter;
#if defined CHICONY_006 || defined FXTOF1

            // Look for temperature sensor
            auto tempSensorIter = std::find_if(
                temperatureSensors.begin(), temperatureSensors.end(),
                [](std::shared_ptr<TemperatureSensorInterface> tSensor) {
                    std::string name;
                    tSensor->getName(name);
                    return name == skChiconyTempSensor;
                });
            if (tempSensorIter == temperatureSensors.end()) {
                DLOG(INFO) << "Could not find " << skChiconyTempSensor
                           << " while looking for temperature sensors for "
                              "camera Chicony";
                break;
            }

            std::shared_ptr<TemperatureSensorInterface> tempSensor =
                *tempSensorIter;

#ifdef CHICONY_006
            std::shared_ptr<Camera> camera =
                std::make_shared<CameraChicony>(dSensor, eeprom, tempSensor);
#elif defined FXTOF1
            std::shared_ptr<Camera> camera =
                std::make_shared<CameraFxTof1>(dSensor, eeprom, tempSensor);
#endif
#else
            std::shared_ptr<Camera> camera = std::make_shared<Camera96Tof1>(
                dSensor, eeprom, afeTempSensor, laserTempSensor);
#endif
            cameras.emplace_back(camera);
            break;
        } // case SensorType::SENSOR_ADDI9036
        } // switch (dSensorDetails.sensorType)
    }

    return cameras;
}

SystemImpl::SystemImpl() {}

SystemImpl::~SystemImpl() = default;

Status SystemImpl::getCameraList(
    std::vector<std::shared_ptr<Camera>> &cameraList) const {

    cameraList.clear();

    // At first, assume SDK is running on target
    std::unique_ptr<SensorEnumeratorInterface> sensorEnumerator =
        SensorEnumeratorFactory::buildTargetSensorEnumerator();
    if (!sensorEnumerator) {
        DLOG(INFO) << "Could not create TargetSensorEnumerator because SDK is "
                      "not running on target.";
        // Assume SDK is running on remote
        sensorEnumerator = SensorEnumeratorFactory::buildUsbSensorEnumerator();
        if (!sensorEnumerator) {
            LOG(ERROR) << "Failed to create UsbSensorEnumerator";
            return Status::GENERIC_ERROR;
        }
    }
    sensorEnumerator->searchSensors();
    cameraList = buildCameras(std::move(sensorEnumerator));

    return Status::OK;
}

Status
SystemImpl::getCameraListAtIp(std::vector<std::shared_ptr<Camera>> &cameraList,
                              const std::string &ip) const {

    cameraList.clear();

    std::unique_ptr<SensorEnumeratorInterface> sensorEnumerator =
        SensorEnumeratorFactory::buildNetworkSensorEnumerator(ip);

    if (!sensorEnumerator) {
        LOG(ERROR) << "Network interface is not enabled."
                      " Please rebuild the SDK "
                      "with the option WITH_NETWORK=on";
        return Status::GENERIC_ERROR;
    }
    sensorEnumerator->searchSensors();
    cameraList = buildCameras(std::move(sensorEnumerator));

    return Status::OK;
}
