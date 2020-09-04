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
#include "server.h"
#include "aditof/aditof.h"
#include "aditof/device_construction_data.h"
#include "aditof/device_enumerator_factory.h"
#include "aditof/device_factory.h"
#include "aditof/eeprom_factory.h"
#include "aditof/eeprom_interface.h"
#include "buffer.pb.h"

#include "../../sdk/src/local_device.h"

#include <glog/logging.h>
#include <iostream>
#include <linux/videodev2.h>
#include <map>
#include <string>
#include <sys/time.h>

using namespace google::protobuf::io;

static int interrupted = 0;

static std::shared_ptr<LocalDevice> device = nullptr;
static std::vector<std::shared_ptr<aditof::EepromInterface>> eeproms;
static payload::ClientRequest buff_recv;
static payload::ServerResponse buff_send;
static std::map<std::string, api_Values> s_map_api_Values;
static void Initialize();
void invoke_sdk_api(payload::ClientRequest buff_recv);
static bool Client_Connected = false;
static bool no_of_client_connected = false;
static unsigned int frame_width = 0;
static unsigned int frame_height = 0;
bool latest_sent_msg_is_was_buffered = false;

struct clientData {
    bool hasFragments;
    std::vector<char> data;
};

static struct lws_protocols protocols[] = {
    {
        "network-protocol",
        Network::callback_function,
        sizeof(clientData),
        RX_BUFFER_BYTES,
    },
    {NULL, NULL, 0, 0} /* terminator */
};

Network ::Network() : context(nullptr) {}

int Network::callback_function(struct lws *wsi,
                               enum lws_callback_reasons reason, void *user,
                               void *in, size_t len) {
    int n;

    switch (reason) {
    case LWS_CALLBACK_ESTABLISHED: {
        /*Check if another client is connected or not*/
        buff_send.Clear();
        if (Client_Connected == false) {
            std::cout << "Conn Established" << std::endl;
            Client_Connected = true;
            buff_send.set_message("Connection Allowed");
            lws_callback_on_writable(wsi);
            break;
        } else {
            std::cout << "Another client connected" << std::endl;
            no_of_client_connected = true;
            buff_send.set_message("Only 1 client connection allowed");
            lws_callback_on_writable(wsi);
        }
        break;
    }

    case LWS_CALLBACK_RECEIVE: {
#ifdef NW_DEBUG
        cout << endl << "Server has received data with len: " << len << endl;
#endif
        const size_t remaining = lws_remaining_packet_payload(wsi);
        bool isFinal = lws_is_final_fragment(wsi);

        struct clientData *clientData = static_cast<struct clientData *>(user);

        if (!remaining && isFinal) {
            if (clientData->hasFragments) {
                // apend message
                char *inData = static_cast<char *>(in);
                clientData->data.insert(clientData->data.end(), inData,
                                        inData + len);
                in = static_cast<void *>(clientData->data.data());
                len = clientData->data.size();
            }

            // process message
            google::protobuf::io::ArrayInputStream ais(in, len);
            CodedInputStream coded_input(&ais);

            buff_recv.ParseFromCodedStream(&coded_input);

            invoke_sdk_api(buff_recv);
            lws_callback_on_writable(wsi);

            clientData->data.clear();
            clientData->hasFragments = false;
        } else {
            // append message
            if (clientData->data.size() == 0) {
                clientData->data.reserve(len + remaining);
            }
            char *inData = static_cast<char *>(in);
            clientData->data.insert(clientData->data.end(), inData,
                                    inData + len);
            clientData->hasFragments = true;
        }

        break;
    }

    case LWS_CALLBACK_SERVER_WRITEABLE: {
        // TO INVESTIGATE: Currently this workaround prevents the server to send
        // the image buffer over and over again but as a side effect it lowers
        // the FPS with about 2-3 frames. How to avoid FPS reduction?
        if (latest_sent_msg_is_was_buffered) {
            latest_sent_msg_is_was_buffered = false;
            break;
        }

        int siz = buff_send.ByteSize();
        unsigned char *pkt =
            new unsigned char[siz + LWS_SEND_BUFFER_PRE_PADDING];
        unsigned char *pkt_pad = pkt + LWS_SEND_BUFFER_PRE_PADDING;
        google::protobuf::io::ArrayOutputStream aos(pkt_pad, siz);
        CodedOutputStream *coded_output = new CodedOutputStream(&aos);
        buff_send.SerializeToCodedStream(coded_output);

        n = lws_write(wsi, pkt_pad, (siz), LWS_WRITE_TEXT);
        if (lws_partial_buffered(wsi)) {
            latest_sent_msg_is_was_buffered = true;
        }
#ifdef NW_DEBUG
        cout << "server is sending " << n << endl;
#endif
        if (n < 0)
            std::cout << "Error Sending" << std::endl;
        else if (n < siz)
            std::cout << "Partial write" << std::endl;
        else if (n == siz) {
#ifdef NW_DEBUG
            cout << "Write successful" << endl;
#endif
        }
        delete coded_output;
        delete[] pkt;
        break;
    }

    case LWS_CALLBACK_CLOSED: {
        if (Client_Connected == true && no_of_client_connected == false) {
            /*CONN_CLOSED event is for first and only client connected*/
            std::cout << "Connection Closed" << std::endl;
            if (device) {
                device.reset();
            }
            Client_Connected = false;
            break;
        } else {
            /*CONN_CLOSED event for more than 1 client connected */
            std::cout << "Another Client Connection Closed" << std::endl;
            no_of_client_connected = false;
            break;
        }
    }

    default: {
#ifdef NW_DEBUG
        cout << reason << endl;
#endif
    } break;
    }

    return 0;
}

void sigint_handler(int) { interrupted = 1; }

std::shared_ptr<aditof::EepromInterface>
findEepromByName(const std::string &name) {
    std::shared_ptr<aditof::EepromInterface> eeprom;

    for (const auto &e : eeproms) {
        std::string eName;
        e->getName(eName);
        if (eName == name) {
            eeprom = e;
            break;
        }
    }

    return eeprom;
}

std::shared_ptr<aditof::EepromInterface>
getEeprom(const ::payload::DeviceConstructionData &data, aditof::Status &status,
          std::string &msg, bool instantiateEeprom = false) {
    std::shared_ptr<aditof::EepromInterface> eeprom;

    // Check if client has sent exactly one name to identify the EEPROM
    if (data.eeproms_size() != 1) {
        msg = "Failed to identify EEPROM. Exactly one name for the eeprom is "
              "accepted. Number of names provided: " +
              std::to_string(data.eeproms_size());
        status = aditof::Status::INVALID_ARGUMENT;

        return eeprom;
    }
    std::string eName = buff_recv.device_data().eeproms(0).driver_name();

    // We lookup the EEPROM instance by name
    eeprom = findEepromByName(eName);
    if (!eeprom) {
        if (instantiateEeprom) {
            // If EEPROM hasn't been instantiated yet, we do it here
            eeprom = std::shared_ptr<aditof::EepromInterface>(
                aditof::EepromFactory::buildEeprom(
                    aditof::ConnectionType::LOCAL));
            if (!eeprom) {
                msg = "Failed to instantiate an EEPROM object";
                status = aditof::Status::GENERIC_ERROR;

                return eeprom;
            }
        }
        msg = "Failed to identify EEPROM with name: " + eName;
        status = aditof::Status::INVALID_ARGUMENT;

        return eeprom;
    }

    msg.clear();
    status = aditof::Status::OK;

    return eeprom;
}

int main(int argc, char *argv[]) {

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));

    info.port = 5000;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;
    info.pt_serv_buf_size = 4096;
    Network *network = new Network();

    network->context = lws_create_context(&info);

    Initialize();

#if 0
  /* Note: Simply enabling this won't work, need libwebsocket compiled differently to demonize this */
  if(lws_daemonize("/tmp/server_lock"))
  {
    fprintf(stderr,"Failed to daemonize\n");
  }
#endif

    while (!interrupted) {
        lws_service(network->context, 0 /* timeout_ms */);
    }

    lws_context_destroy(network->context);
    delete network;

    return 0;
}

void invoke_sdk_api(payload::ClientRequest buff_recv) {
    buff_send.Clear();
    buff_send.set_server_status(::payload::ServerStatus::REQUEST_ACCEPTED);

    switch (s_map_api_Values[buff_recv.func_name()]) {

    case FIND_DEVICES: {
        DLOG(INFO) << "FindDevices function\n";

        std::vector<aditof::DeviceConstructionData> devicesInfo;
        auto localDevEnumerator =
            aditof::DeviceEnumeratorFactory::buildDeviceEnumerator();

        aditof::Status status = localDevEnumerator->findDevices(devicesInfo);
        for (const auto &devInfo : devicesInfo) {
            auto pbDevInfo = buff_send.add_device_info();
            pbDevInfo->set_device_type(static_cast<::payload::DeviceType>(
                aditof::ConnectionType::ETHERNET));
            pbDevInfo->set_driver_path(devInfo.driverPath);
            for (const auto &eepromInfo : devInfo.eeproms) {
                auto pbEepromInfo = pbDevInfo->add_eeproms();
                pbEepromInfo->set_driver_name(eepromInfo.driverName);
                pbEepromInfo->set_driver_path(eepromInfo.driverPath);
            }
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case INSTANTIATE_DEVICE: {
        DLOG(INFO) << "InstantiateDevice function\n";

        aditof::Status status = aditof::Status::OK;
        std::string errMsg;
        aditof::DeviceConstructionData devData;

        devData.connectionType = aditof::ConnectionType::LOCAL;
        devData.driverPath = buff_recv.device_data().driver_path();
        std::shared_ptr<aditof::DeviceInterface> deviceI =
            aditof::DeviceFactory::buildDevice(devData);
        device = std::dynamic_pointer_cast<LocalDevice>(deviceI);
        if (!device) {
            errMsg = "Failed to create local device";
            status = aditof::Status::INVALID_ARGUMENT;
        } else {
            aditof::DeviceDetails devDetails;
            deviceI->getDetails(devDetails);
            buff_send.set_sensor_type(
                static_cast<::payload::SensorType>(devDetails.sensorType));
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        if (!errMsg.empty())
            buff_send.set_message(errMsg);
        break;
    }

    case DESTROY_DEVICE: {
        DLOG(INFO) << "DestroyDevice function\n";

        if (device) {
            device.reset();
        }
        break;
    }

    case OPEN: {
        DLOG(INFO) << "Open function\n";

        aditof::Status status = device->open();
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case START: {
        DLOG(INFO) << "Start function\n";

        aditof::Status status = device->start();
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STOP: {
        DLOG(INFO) << "Stop function\n";

        aditof::Status status = device->stop();
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_AVAILABLE_FRAME_TYPES: {
        DLOG(INFO) << "GetAvailableFrameTypes function";

        std::vector<aditof::FrameDetails> frameDetails;
        aditof::Status status = device->getAvailableFrameTypes(frameDetails);
        for (auto detail : frameDetails) {
            auto type = buff_send.add_available_frame_types();
            type->set_width(detail.width);
            type->set_height(detail.height);
            type->set_type(detail.type);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_FRAME_TYPE: {
        DLOG(INFO) << "SetFrameType function\n";

        aditof::FrameDetails details;
        details.width = buff_recv.frame_type().width();
        details.height = buff_recv.frame_type().height();
        details.type = buff_recv.frame_type().type();
        aditof::Status status = device->setFrameType(details);
        buff_send.set_status(static_cast<::payload::Status>(status));

        frame_width = details.width;
        frame_height = details.height;
        break;
    }

    case PROGRAM: {
        DLOG(INFO) << "Program function\n";

        size_t programSize = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint8_t *pdata = reinterpret_cast<const uint8_t *>(
            buff_recv.func_bytes_param(0).c_str());
        aditof::Status status = device->program(pdata, programSize);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_FRAME: {
        DLOG(INFO) << "GetFrame function\n";

        aditof::Status status = device->waitForBuffer();
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        struct v4l2_buffer buf;

        status = device->dequeueInternalBuffer(buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        unsigned int buf_data_len;
        uint8_t *buffer;

        status = device->getInternalBuffer(&buffer, buf_data_len, buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        buff_send.add_bytes_payload(buffer, buf_data_len * sizeof(uint8_t));

        status = device->enqueueInternalBuffer(buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        buff_send.set_status(payload::Status::OK);
        break;
    }

    case READ_AFE_REGISTERS: {
        DLOG(INFO) << "ReadAfeRegisters function\n";

        size_t length = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(0).c_str());
        uint16_t *data = new uint16_t[length];
        aditof::Status status = device->readAfeRegisters(address, data, length);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(data, length * sizeof(uint16_t));
        }
        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case WRITE_AFE_REGISTERS: {
        DLOG(INFO) << "WriteAfeRegisters function\n";

        size_t length = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(0).c_str());
        const uint16_t *data = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(1).c_str());
        aditof::Status status =
            device->writeAfeRegisters(address, data, length);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case READ_AFE_TEMP: {
        DLOG(INFO) << "ReadAfeTemp function\n";

        float temperature;
        aditof::Status status = device->readAfeTemp(temperature);
        if (status == aditof::Status::OK) {
            buff_send.add_float_payload(temperature);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case READ_LASER_TEMP: {
        DLOG(INFO) << "ReadLaserTemp function\n";

        float temperature;
        aditof::Status status = device->readLaserTemp(temperature);
        if (status == aditof::Status::OK) {
            buff_send.add_float_payload(temperature);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case EEPROM_OPEN: {
        DLOG(INFO) << "EepromOpen function\n";

        aditof::Status status;
        std::string msg;
        auto eeprom = getEeprom(buff_recv.device_data(), status, msg, true);
        if (status != aditof::Status::OK) {
            buff_send.set_message(msg);
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        status = eeprom->open(nullptr,
                              buff_recv.device_data().eeproms(0).driver_name(),
                              buff_recv.device_data().eeproms(0).driver_path());
        if (status == aditof::Status::OK) {
            eeproms.push_back(eeprom);
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case EEPROM_READ: {
        DLOG(INFO) << "EepromRead function\n";

        aditof::Status status;
        std::string msg;
        auto eeprom = getEeprom(buff_recv.device_data(), status, msg);
        if (status != aditof::Status::OK) {
            buff_send.set_message(msg);
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        uint32_t address = static_cast<uint32_t>(buff_recv.func_int32_param(0));
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(1));
        uint8_t *buffer = new uint8_t[length];
        status = eeprom->read(address, buffer, length);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(buffer, length);
        }
        delete[] buffer;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case EEPROM_WRITE: {
        DLOG(INFO) << "Eepromrite function\n";

        aditof::Status status;
        std::string msg;
        auto eeprom = getEeprom(buff_recv.device_data(), status, msg);
        if (status != aditof::Status::OK) {
            buff_send.set_message(msg);
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        uint32_t address = static_cast<uint32_t>(buff_recv.func_int32_param(0));
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(1));
        const uint8_t *buffer = reinterpret_cast<const uint8_t *>(
            buff_recv.func_bytes_param(0).c_str());

        status = eeprom->write(address, buffer, length);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case EEPROM_CLOSE: {
        DLOG(INFO) << "EepromClose function\n";

        aditof::Status status;
        std::string msg;
        auto eeprom = getEeprom(buff_recv.device_data(), status, msg);
        if (status != aditof::Status::OK) {
            buff_send.set_message(msg);
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        status = eeprom->close();

        if (status == aditof::Status::OK) {
            eeproms.erase(std::remove(eeproms.begin(), eeproms.end(), eeprom),
                          eeproms.end());
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    default: {
        std::string msgErr = "Function not found";
        std::cout << msgErr << "\n";

        buff_send.set_message(msgErr);
        buff_send.set_server_status(::payload::ServerStatus::REQUEST_UNKNOWN);
        break;
    }
    } // switch

    buff_recv.Clear();
}

void Initialize() {
    s_map_api_Values["FindDevices"] = FIND_DEVICES;
    s_map_api_Values["InstantiateDevice"] = INSTANTIATE_DEVICE;
    s_map_api_Values["DestroyDevice"] = DESTROY_DEVICE;
    s_map_api_Values["Open"] = OPEN;
    s_map_api_Values["Start"] = START;
    s_map_api_Values["Stop"] = STOP;
    s_map_api_Values["GetAvailableFrameTypes"] = GET_AVAILABLE_FRAME_TYPES;
    s_map_api_Values["SetFrameType"] = SET_FRAME_TYPE;
    s_map_api_Values["Program"] = PROGRAM;
    s_map_api_Values["GetFrame"] = GET_FRAME;
    s_map_api_Values["ReadAfeRegisters"] = READ_AFE_REGISTERS;
    s_map_api_Values["WriteAfeRegisters"] = WRITE_AFE_REGISTERS;
    s_map_api_Values["ReadAfeTemp"] = READ_AFE_TEMP;
    s_map_api_Values["ReadLaserTemp"] = READ_LASER_TEMP;
    s_map_api_Values["EepromOpen"] = EEPROM_OPEN;
    s_map_api_Values["EepromRead"] = EEPROM_READ;
    s_map_api_Values["EepromWrite"] = EEPROM_WRITE;
    s_map_api_Values["EepromClose"] = EEPROM_CLOSE;
}
