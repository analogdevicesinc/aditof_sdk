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
#include "network_depth_sensor.h"
#include "connections/network/network.h"
#include "device_utils.h"

#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <unordered_map>

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};
struct NetworkDepthSensor::ImplData {
    NetworkHandle *handle;
    bool ownsHandle;
    std::string ip;
    aditof::FrameDetails frameDetails_cache;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
    bool opened;
};

NetworkDepthSensor::NetworkDepthSensor(const std::string &name, int id,
                                       const std::string &ip)
    : m_sensorName(name), m_id(id),
      m_implData(new NetworkDepthSensor::ImplData) {

    m_implData->handle = new NetworkHandle;
    m_implData->ownsHandle = true;
    m_implData->handle->net = new Network();
    m_implData->ip = ip;
    m_implData->opened = false;
}

NetworkDepthSensor::NetworkDepthSensor(const std::string &name, int id,
                                       void *handle)
    : m_sensorName(name), m_id(id),
      m_implData(new NetworkDepthSensor::ImplData) {

    m_implData->handle = reinterpret_cast<struct NetworkHandle *>(handle);
    m_implData->ownsHandle = false;
    m_implData->opened = false;
}

NetworkDepthSensor::~NetworkDepthSensor() {

    if (!m_implData->handle->net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
    }

    if (m_implData->ownsHandle) {
        {
            std::unique_lock<std::mutex> mutex_lock(
                m_implData->handle->net_mutex);
            m_implData->handle->net->send_buff.set_func_name("HangUp");
            m_implData->handle->net->send_buff.set_expect_reply(false);

            if (m_implData->handle->net->SendCommand() != 0) {
                LOG(WARNING) << "Send Command Failed";
            }
        } // mutex_lock goes out of scope and the mutex that it owns can be destroyed

        delete m_implData->handle->net;
        delete m_implData->handle;
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }
}

aditof::Status NetworkDepthSensor::open() {
    using namespace aditof;

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (m_implData->ownsHandle && net->ServerConnect(m_implData->ip) != 0) {
        LOG(WARNING) << "Server Connect Failed";
        return Status::UNREACHABLE;
    }

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("Open");
    net->send_buff.mutable_sensors_info()->add_image_sensors()->set_id(m_id);
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

    Status status = static_cast<Status>(net->recv_buff.status());

    if (status == Status::OK) {
        m_implData->opened = true;
    }

    return status;
}

aditof::Status NetworkDepthSensor::start() {
    using namespace aditof;

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("Start");
    net->send_buff.mutable_sensors_info()->add_image_sensors()->set_id(m_id);
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

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status NetworkDepthSensor::stop() {
    using namespace aditof;

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    LOG(INFO) << "Stopping device";

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("Stop");
    net->send_buff.mutable_sensors_info()->add_image_sensors()->set_id(m_id);
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

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status NetworkDepthSensor::getAvailableFrameTypes(
    std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("GetAvailableFrameTypes");
    net->send_buff.mutable_sensors_info()->add_image_sensors()->set_id(m_id);
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

    // Cleanup array (if required) before filling it with the available types
    if (types.size() != 0) {
        types.clear();
    }

    for (int i = 0; i < net->recv_buff.available_frame_types_size(); i++) {
        payload::FrameDetails details = net->recv_buff.available_frame_types(i);
        aditof::FrameDetails aditofDetails;
        aditofDetails.width = details.width();
        aditofDetails.height = details.height();
        aditofDetails.type = details.type();
        aditofDetails.fullDataWidth = details.full_data_width();
        aditofDetails.fullDataHeight = details.full_data_height();
        aditofDetails.rgbWidth = details.rgb_width();
        aditofDetails.rgbHeight = details.rgb_height();
        types.push_back(aditofDetails);
    }

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status
NetworkDepthSensor::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("SetFrameType");
    net->send_buff.mutable_sensors_info()->add_image_sensors()->set_id(m_id);
    net->send_buff.mutable_frame_type()->set_width(details.width);
    net->send_buff.mutable_frame_type()->set_height(details.height);
    net->send_buff.mutable_frame_type()->set_type(details.type);
    net->send_buff.mutable_frame_type()->set_full_data_width(
        details.fullDataWidth);
    net->send_buff.mutable_frame_type()->set_full_data_height(
        details.fullDataHeight);
    net->send_buff.mutable_frame_type()->set_rgb_width(details.rgbWidth);
    net->send_buff.mutable_frame_type()->set_rgb_height(details.rgbHeight);
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

    Status status = static_cast<Status>(net->recv_buff.status());

    if (status == Status::OK) {
        m_implData->frameDetails_cache = details;
    }

    return status;
}

aditof::Status NetworkDepthSensor::program(const uint8_t *firmware,
                                           size_t size) {
    using namespace aditof;

    if (firmware == nullptr) {
        LOG(ERROR) << "Received firmware null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(size > 0);

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("Program");
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(size));
    net->send_buff.add_func_bytes_param(firmware, size);
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

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status NetworkDepthSensor::getFrame(uint16_t *buffer,
                                            aditof::BufferInfo *bufferInfo) {
    using namespace aditof;

    if (buffer == nullptr) {
        LOG(ERROR) << "Received buffer null pointer";
        return Status::INVALID_ARGUMENT;
    }

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("GetFrame");
    net->send_buff.mutable_sensors_info()->add_image_sensors()->set_id(m_id);
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

    Status status = static_cast<Status>(net->recv_buff.status());
    if (status != Status::OK) {
        LOG(WARNING) << "getFrame() failed on target";
        return status;
    }

    memcpy(buffer, net->recv_buff.bytes_payload(0).c_str(),
           net->recv_buff.bytes_payload(0).length());

    if (bufferInfo) {
        bufferInfo->timestamp = net->recv_buff.buffer_details().timestamp();
    }
    return status;
}

aditof::Status NetworkDepthSensor::readAfeRegisters(const uint16_t *address,
                                                    uint16_t *data,
                                                    size_t length) {
    using namespace aditof;

    if (address == nullptr) {
        LOG(ERROR) << "Received AfeRegisters address null pointer";
        return Status::INVALID_ARGUMENT;
    }

    if (data == nullptr) {
        LOG(ERROR) << "Received AfeRegisters data null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(length > 0);

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("ReadAfeRegisters");
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(length));
    net->send_buff.add_func_bytes_param(address, length * sizeof(uint16_t));
    net->send_buff.add_func_bytes_param(data, length * sizeof(uint16_t));
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

    Status status = static_cast<Status>(net->recv_buff.status());

    if (status == Status::OK) {
        memcpy(data, net->recv_buff.bytes_payload(0).c_str(),
               net->recv_buff.bytes_payload(0).length());
    }

    return status;
}

aditof::Status NetworkDepthSensor::writeAfeRegisters(const uint16_t *address,
                                                     const uint16_t *data,
                                                     size_t length) {
    using namespace aditof;

    if (address == nullptr) {
        LOG(ERROR) << "Received AfeRegisters address null pointer";
        return Status::INVALID_ARGUMENT;
    }

    if (data == nullptr) {
        LOG(ERROR) << "Received AfeRegisters data null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(length > 0);

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("WriteAfeRegisters");
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(length));
    net->send_buff.add_func_bytes_param(address, length * sizeof(uint16_t));
    net->send_buff.add_func_bytes_param(data, length * sizeof(uint16_t));
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

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status
NetworkDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;

    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::getHandle(void **handle) {
    *handle = m_implData->handle;

    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::getName(std::string &sensorName) const {
    sensorName = m_sensorName;

    return aditof::Status::OK;
}
