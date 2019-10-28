#include "ethernet_device.h"
#include "device_utils.h"
#include "network.h"
#include "utils.h"

#include <glog/logging.h>
#include <unordered_map>

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};

struct EthernetDevice::ImplData {
    Network *net;
    std::string ip;
    aditof::FrameDetails frameDetails_cache;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
    std::mutex net_mutex;
};

EthernetDevice::EthernetDevice(const aditof::DeviceConstructionData &data)
    : m_implData(new EthernetDevice::ImplData) {

    Network *net = new Network();
    m_implData->net = net;
    m_implData->ip = data.ip;

    m_deviceDetails.sensorType = aditof::SensorType::SENSOR_96TOF1;

    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    /* Make connection with LWS server running on Dragonboard */
    if (net->ServerConnect(m_implData->ip) != 0) {
        LOG(WARNING) << "Server Connect Failed";
    }

    net->send_buff.set_func_name("InstantiateDevice");
    net->send_buff.mutable_device_data()->set_driver_path(data.driverPath);
    net->send_buff.set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
    }

    if (net->recv_buff.server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
    }

    auto status = static_cast<aditof::Status>(net->recv_buff.status());
    if (status != aditof::Status::OK) {
        LOG(WARNING) << "API execution on Target Failed with error: "
                     << static_cast<int>(status);
    }
}

EthernetDevice::~EthernetDevice() {
    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
    }

    net->send_buff.set_func_name("DestroyDevice");
    net->send_buff.set_expect_reply(false);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
    }

    delete m_implData->net;

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }
}

aditof::Status EthernetDevice::open() {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("Open");
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

aditof::Status EthernetDevice::start() {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("Start");
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

aditof::Status EthernetDevice::stop() {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    LOG(INFO) << "Stopping device";

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("Stop");
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

aditof::Status EthernetDevice::getAvailableFrameTypes(
    std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("GetAvailableFrameTypes");
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
        aditofDetails.cal_data.gain = details.cal_data().gain();
        aditofDetails.cal_data.offset = details.cal_data().offset();

        types.push_back(aditofDetails);
    }

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status
EthernetDevice::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("SetFrameType");
    net->send_buff.mutable_frame_type()->set_width(details.width);
    net->send_buff.mutable_frame_type()->set_height(details.height);
    net->send_buff.mutable_frame_type()->set_type(details.type);
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

aditof::Status EthernetDevice::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

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

aditof::Status EthernetDevice::getFrame(uint16_t *buffer) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("GetFrame");
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

    // Deinterleave data. The server sends raw data (uninterleaved) for better
    // throughput (raw data chunck is smaller, deinterleaving is usually slower
    // on target).

    aditof::deinterleave(net->recv_buff.bytes_payload(0).c_str(), buffer,
                         net->recv_buff.bytes_payload(0).length(),
                         m_implData->frameDetails_cache.width,
                         m_implData->frameDetails_cache.height);

    return status;
}

aditof::Status EthernetDevice::readEeprom(uint32_t address, uint8_t *data,
                                          size_t length) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("ReadEeprom");
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(address));
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(length));
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

    memcpy(data, net->recv_buff.bytes_payload(0).c_str(),
           net->recv_buff.bytes_payload(0).length());

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status EthernetDevice::writeEeprom(uint32_t address,
                                           const uint8_t *data, size_t length) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("WriteEeprom");
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(address));
    net->send_buff.add_func_int32_param(static_cast<::google::int32>(length));
    net->send_buff.add_func_bytes_param(data, length);
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

aditof::Status EthernetDevice::readAfeRegisters(const uint16_t *address,
                                                uint16_t *data, size_t length) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

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

    memcpy(data, net->recv_buff.bytes_payload(0).c_str(),
           net->recv_buff.bytes_payload(0).length());

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status EthernetDevice::writeAfeRegisters(const uint16_t *address,
                                                 const uint16_t *data,
                                                 size_t length) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

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

aditof::Status EthernetDevice::readAfeTemp(float &temperature) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("ReadAfeTemp");
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

    temperature = net->recv_buff.float_payload(0);

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status EthernetDevice::readLaserTemp(float &temperature) {
    using namespace aditof;

    Network *net = m_implData->net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff.set_func_name("ReadLaserTemp");
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

    temperature = net->recv_buff.float_payload(0);

    Status status = static_cast<Status>(net->recv_buff.status());

    return status;
}

aditof::Status EthernetDevice::setCalibrationParams(const std::string &mode,
                                                    float gain, float offset,
                                                    int range) {

    const int16_t pixelMaxValue = (1 << 12) - 1; // 4095
    CalibrationData calib_data;
    calib_data.mode = mode;
    calib_data.gain = gain;
    calib_data.offset = offset;
    calib_data.cache = aditof::Utils::buildCalibrationCache(
        gain, offset, pixelMaxValue, range);
    m_implData->calibration_cache[mode] = calib_data;

    return aditof::Status::OK;
}

aditof::Status
EthernetDevice::applyCalibrationToFrame(uint16_t *frame,
                                        const std::string &mode) {
    // This should be done on the target by the LocalDevice since the
    // EthernetDevice is is actually an implementation of the communication
    // channel. But it is far more efficient to do this on the host which may
    // have more computing power than the target and also avoids the overhead of
    // transporting the frame back and forth.

    float gain = m_implData->calibration_cache[mode].gain;
    float offset = m_implData->calibration_cache[mode].offset;

    if (m_implData->frameDetails_cache.type.empty()) {
        LOG(WARNING) << "Frame type has not been set for this device";
        return aditof::Status::GENERIC_ERROR;
    }

    unsigned int width = m_implData->frameDetails_cache.width;
    unsigned int height = m_implData->frameDetails_cache.height;

    aditof::Utils::calibrateFrame(m_implData->calibration_cache[mode].cache,
                                  frame, width, height);

    return aditof::Status::OK;
}

aditof::Status
EthernetDevice::getDetails(aditof::DeviceDetails &details) const {
    details = m_deviceDetails;

    return aditof::Status::OK;
}
