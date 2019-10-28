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

        tofData.deviceType =
            static_cast<aditof::DeviceType>(pbData.device_type());
        tofData.driverPath = pbData.driver_path();
        tofData.ip = m_ip;
        devices.push_back(tofData);
    }
    status = static_cast<Status>(net->recv_buff.status());

    return status;
}
