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

#ifndef TARGET
#include "cameras/3d-smart-camera/camera_3d_smart.h"
#include "cameras/ad-96tof1-ebz/camera_96tof1.h"
#include "cameras/ad-fxtof1-ebz/camera_fxtof1.h"
#else
#if defined(FXTOF1)
#include "cameras/ad-fxtof1-ebz/camera_fxtof1.h"
#elif defined(SMART_3D)
#include "cameras/3d-smart-camera/camera_3d_smart.h"
#else
#include "cameras/ad-96tof1-ebz/camera_96tof1.h"
#endif
#endif // #ifndef TARGET

#include <aditof/camera.h>
#include <algorithm>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

#include "aditof/version.h"

#ifdef HAS_NETWORK
#include <lws_config.h>
#endif

using namespace aditof;

/* Constructs a camera based on the sensors & other devices provided
   found by the sensor enumerator. */
static std::shared_ptr<Camera>
buildCamera(std::unique_ptr<SensorEnumeratorInterface> enumerator) {
    std::vector<std::shared_ptr<DepthSensorInterface>> depthSensors;
    std::vector<std::shared_ptr<StorageInterface>> storages;
    std::vector<std::shared_ptr<TemperatureSensorInterface>> temperatureSensors;
    std::shared_ptr<Camera> camera;
    CameraType cameraType;

    enumerator->getDepthSensors(depthSensors);
    if (depthSensors.size() < 1) {
        LOG(ERROR) << "No imagers found";
        return nullptr;
    }

    enumerator->getStorages(storages);
    enumerator->getTemperatureSensors(temperatureSensors);
    enumerator->getCameraTypeOnTarget(cameraType);

#ifndef TARGET
    switch (cameraType) {
    case CameraType::AD_96TOF1_EBZ:
        camera = std::make_shared<Camera96Tof1>(depthSensors[0], storages,
                                                temperatureSensors);
        break;
    case CameraType::AD_FXTOF1_EBZ:
        camera = std::make_shared<CameraFxTof1>(depthSensors[0], storages,
                                                temperatureSensors);
        break;
    case CameraType::SMART_3D_CAMERA:
        camera = std::make_shared<Camera3D_Smart>(depthSensors[0], storages,
                                                  temperatureSensors);
        break;
    }
#else
#if defined(FXTOF1)
    camera = std::make_shared<CameraFxTof1>(depthSensors[0], storages,
                                            temperatureSensors);
#elif defined(SMART_3D)
    camera = std::make_shared<Camera3D_Smart>(depthSensors[0], storages,
                                              temperatureSensors);
#else
    camera = std::make_shared<Camera96Tof1>(depthSensors[0], storages,
                                            temperatureSensors);
#endif
#endif // #ifndef TARGET

    return camera;
}

SystemImpl::SystemImpl() {
    static bool sdkRevisionLogged = false;
    if (!sdkRevisionLogged) {
        LOG(INFO) << "SDK version: " << aditof::getApiVersion()
                  << " | branch: " << ADITOFSDK_GIT_BRANCH
                  << " | commit: " << ADITOFSDK_GIT_COMMIT;
        sdkRevisionLogged = true;
#if HAS_NETWORK
        LOG(INFO) << "SDK built with websockets version:"
                  << LWS_LIBRARY_VERSION;
#endif
    }
}

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
    Status status = sensorEnumerator->searchSensors();
    if (status == Status::OK) {
        auto camera = buildCamera(std::move(sensorEnumerator));
        if (camera) {
            cameraList.emplace_back(camera);
        }
    }

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
    Status status = sensorEnumerator->searchSensors();
    if (status == Status::OK) {
        auto camera = buildCamera(std::move(sensorEnumerator));
        if (camera) {
            cameraList.emplace_back(camera);
        }
    }

    return Status::OK;
}
