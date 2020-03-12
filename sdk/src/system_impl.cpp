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
#include "camera_factory.h"

#include <aditof/camera.h>
#include <aditof/device_construction_data.h>
#include <aditof/device_enumerator_factory.h>
#include <aditof/device_factory.h>

#include <glog/logging.h>

SystemImpl::SystemImpl()
    : m_enumerator(aditof::DeviceEnumeratorFactory::buildDeviceEnumerator()) {}

SystemImpl::~SystemImpl() = default;

aditof::Status SystemImpl::initialize() {
    using namespace aditof;
    Status status = Status::OK;

    std::vector<aditof::DeviceConstructionData> devsData;
    m_enumerator->findDevices(devsData);

    for (const auto &data : devsData) {
        std::unique_ptr<DeviceInterface> device =
            DeviceFactory::buildDevice(data);
        std::shared_ptr<Camera> camera =
            CameraFactory::buildCamera(std::move(device));
        m_cameras.emplace_back(camera);
    }

    LOG(INFO) << "System initialized";

    return status;
}

aditof::Status SystemImpl::getCameraList(
    std::vector<std::shared_ptr<aditof::Camera>> &cameraList) const {
    using namespace aditof;
    Status status = Status::OK;

    cameraList = m_cameras;

    return status;
}

aditof::Status SystemImpl::getCameraListAtIp(
    std::vector<std::shared_ptr<aditof::Camera>> &cameraList,
    const std::string &ip) const {
    using namespace aditof;
    Status status = Status::OK;

    cameraList.clear();

    std::vector<aditof::DeviceConstructionData> devsData;
    auto ethernetEnumerator =
        DeviceEnumeratorFactory::buildDeviceEnumeratorEthernet(ip);
    status = ethernetEnumerator->findDevices(devsData);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get find devices on target with ip: " << ip;
        return status;
    }

    for (const auto &data : devsData) {
        std::unique_ptr<DeviceInterface> device =
            DeviceFactory::buildDevice(data);
        std::shared_ptr<Camera> camera =
            CameraFactory::buildCamera(std::move(device));
        cameraList.emplace_back(camera);
    }

    return Status::OK;
}
