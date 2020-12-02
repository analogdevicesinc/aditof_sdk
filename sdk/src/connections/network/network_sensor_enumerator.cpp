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
#include "network_sensor_enumerator.h"
#include "connections/network/network.h"
#include "connections/network/network_depth_sensor.h"
#include "connections/network/network_storage.h"
#include "connections/network/network_temperature_sensor.h"

#include <glog/logging.h>

using namespace aditof;

NetworkSensorEnumerator::NetworkSensorEnumerator(const std::string &ip)
    : m_ip(ip) {}

NetworkSensorEnumerator::~NetworkSensorEnumerator() = default;

Status NetworkSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    LOG(INFO) << "Looking for devices over ethernet";

    std::unique_ptr<Network> net(new Network());

    if (net->ServerConnect(m_ip) != 0) {
        LOG(WARNING) << "Server Connect Failed";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("FindDevices");
    net->send_buff.set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff.server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }
    const payload::ServerResponse &msg = net->recv_buff;
    for (int i = 0; i < msg.device_info().size(); ++i) {
        const payload::DeviceConstructionData &pbData = msg.device_info(i);

        m_sensorsType.emplace_back(
            static_cast<aditof::SensorType>(pbData.device_type()));

        for (int j = 0; j < pbData.eeproms().size(); ++j) {
            const payload::EepromConstructionData &pbEepromData =
                pbData.eeproms(j);
            m_storagesInfo.emplace_back(pbEepromData.driver_name());
        }

        for (int k = 0; k < pbData.temperature_sensors_size(); ++k) {
            m_tempeatureSensorsInfo.emplace_back(pbData.temperature_sensors(k));
        }
    }
    status = static_cast<Status>(net->recv_buff.status());

    return status;
}

Status NetworkSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (auto sensorType : m_sensorsType) {
        auto sensor = std::make_shared<NetworkDepthSensor>(m_ip, sensorType);
        depthSensors.emplace_back(sensor);
    }

    return Status::OK;
}

Status NetworkSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> &storages) {

    storages.clear();

    for (const auto &name : m_storagesInfo) {
        auto storage = std::make_shared<NetworkStorage>(name);
        storages.emplace_back(storage);
    }

    return Status::OK;
}

Status NetworkSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        &temperatureSensors) {

    temperatureSensors.clear();

    for (const auto &name : m_tempeatureSensorsInfo) {
        auto tSensor = std::make_shared<NetworkTemperatureSensor>(name);
        temperatureSensors.emplace_back(tSensor);
    }

    return Status::OK;
}