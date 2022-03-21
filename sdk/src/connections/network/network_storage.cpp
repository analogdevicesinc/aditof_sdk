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
#include "connections/network/network_storage.h"
#include "connections/network/network.h"

#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

using namespace aditof;

struct NetworkStorage::ImplData {
    NetworkHandle *handle;
    std::string name;
    unsigned int id;
};

NetworkStorage::NetworkStorage(const std::string &name, unsigned int id)
    : m_implData(new NetworkStorage::ImplData) {
    m_implData->name = name;
    m_implData->id = id;
}

NetworkStorage::~NetworkStorage() = default;

Status NetworkStorage::open(void *handle) {
    if (!handle) {
        LOG(ERROR) << "Invalid handle";
        return Status::INVALID_ARGUMENT;
    }
    m_implData->handle = reinterpret_cast<struct NetworkHandle *>(handle);

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("StorageOpen");
    net->send_buff.add_func_int32_param(m_implData->id);
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

Status NetworkStorage::read(const uint32_t address, uint8_t *data,
                            const size_t bytesCount) {

    if (data == nullptr) {
        LOG(ERROR) << "Received data null pointer";
        return aditof::Status::INVALID_ARGUMENT;
    }

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("StorageRead");
    net->send_buff.add_func_int32_param(m_implData->id);
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(address));
    net->send_buff.add_func_int32_param(
        static_cast<::google::int32>(bytesCount));
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

Status NetworkStorage::write(const uint32_t address, const uint8_t *data,
                             const size_t bytesCount) {

    if (data == nullptr) {
        LOG(ERROR) << "Received data null pointer";
        return aditof::Status::INVALID_ARGUMENT;
    }

    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("StorageWrite");
    net->send_buff.add_func_int32_param(m_implData->id);
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(address));
    net->send_buff.add_func_int32_param(
        static_cast<::google::int32>(bytesCount));
    net->send_buff.add_func_bytes_param(data, bytesCount);
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

Status NetworkStorage::close() {
    Network *net = m_implData->handle->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("StorageClose");
    net->send_buff.add_func_int32_param(m_implData->id);
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
        m_implData->handle = nullptr;
    }

    return status;
}

Status NetworkStorage::getName(std::string &name) const {
    name = m_implData->name;
    return Status::OK;
}
