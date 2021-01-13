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
#if defined(CHICONY_006)
#include "camera_chicony_006.h"
#elif defined(FXTOF1)
#include "camera_fxtof1.h"
#else
#include "camera_96tof1.h"
#endif

#include <aditof/camera.h>
#include <algorithm>
#include <glog/logging.h>

using namespace aditof;

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
#if defined(CHICONY_006)
        std::shared_ptr<Camera> camera = std::make_shared<CameraChicony>(
            dSensor, storages, temperatureSensors);
#elif defined(FXTOF1)
        std::shared_ptr<Camera> camera = std::make_shared<CameraFxTof1>(
            dSensor, storages, temperatureSensors);
#else
        std::shared_ptr<Camera> camera = std::make_shared<Camera96Tof1>(
            dSensor, storages, temperatureSensors);
#endif
        cameras.emplace_back(camera);
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
                      "Please rebuild the SDK "
                      "with the option WITH_NETWORK=on";
        return Status::GENERIC_ERROR;
    }
    sensorEnumerator->searchSensors();
    cameraList = buildCameras(std::move(sensorEnumerator));

    return Status::OK;
}
