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
#include "device_enumerator_ethernet.h"
#include "network.h"

#include <glog/logging.h>

DeviceEnumeratorEthernet::DeviceEnumeratorEthernet(const std::string &ip)
    : m_ip(ip) {}

DeviceEnumeratorEthernet::~DeviceEnumeratorEthernet() = default;

aditof::Status DeviceEnumeratorEthernet::findDevices(
    std::vector<aditof::DeviceConstructionData> &devices) {
    using namespace aditof;
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
        aditof::DeviceConstructionData tofData;

        tofData.connectionType =
            static_cast<aditof::ConnectionType>(pbData.device_type());
        tofData.driverPath = pbData.driver_path();
        tofData.ip = m_ip;

        for (int j = 0; j < pbData.eeproms().size(); ++j) {
            const payload::EepromConstructionData &pbEepromData =
                pbData.eeproms(j);
            aditof::EepromConstructionData tofEepromData;
            tofEepromData.driverName = pbEepromData.driver_name();
            tofEepromData.driverPath = pbEepromData.driver_path();
            tofData.eeproms.push_back(tofEepromData);
        }

        devices.push_back(tofData);
    }
    status = static_cast<Status>(net->recv_buff.status());

    return status;
}
