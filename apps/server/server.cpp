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
#include "aditof/sensor_enumerator_factory.h"
#include "aditof/sensor_enumerator_interface.h"
#include "buffer.pb.h"

#include "../../sdk/src/connections/target/v4l_buffer_access_interface.h"

#include <glog/logging.h>
#include <iostream>
#include <linux/videodev2.h>
#include <map>
#include <string>
#include <sys/time.h>

using namespace google::protobuf::io;

static int interrupted = 0;

/* Available sensors */
static std::vector<std::shared_ptr<aditof::StorageInterface>> storages;
static std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
    temperatureSensors;
bool sensors_are_created = false;

/* Server only works with one depth sensor */
std::shared_ptr<aditof::DepthSensorInterface> camDepthSensor;
std::shared_ptr<aditof::V4lBufferAccessInterface> sensorV4lBufAccess;

static payload::ClientRequest buff_recv;
static payload::ServerResponse buff_send;
static std::map<std::string, api_Values> s_map_api_Values;
static void Initialize();
void invoke_sdk_api(payload::ClientRequest buff_recv);
static bool Client_Connected = false;
static bool no_of_client_connected = false;
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

static void cleanup_sensors() {
    for (auto &storage : storages) {
        storage->close();
    }
    storages.clear();
    for (auto &sensor : temperatureSensors) {
        sensor->close();
    }
    temperatureSensors.clear();
    sensorV4lBufAccess.reset();
    camDepthSensor.reset();

    sensors_are_created = false;
}

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
    std::unique_ptr<Network> network(new Network);

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

    if (sensors_are_created) {
        cleanup_sensors();
    }

    lws_context_destroy(network->context);

    return 0;
}

void invoke_sdk_api(payload::ClientRequest buff_recv) {
    buff_send.Clear();
    buff_send.set_server_status(::payload::ServerStatus::REQUEST_ACCEPTED);

    DLOG(INFO) << buff_recv.func_name() << " function";

    switch (s_map_api_Values[buff_recv.func_name()]) {

    case FIND_SENSORS: {
        std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;

        if (!sensors_are_created) {
            auto sensorsEnumerator =
                aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
            if (!sensorsEnumerator) {
                std::string errMsg =
                    "Failed to create a target sensor enumerator";
                LOG(WARNING) << errMsg;
                buff_send.set_message(errMsg);
                buff_send.set_status(static_cast<::payload::Status>(
                    aditof::Status::UNAVAILABLE));
                break;
            }

            sensorsEnumerator->searchSensors();
            sensorsEnumerator->getDepthSensors(depthSensors);
            sensorsEnumerator->getStorages(storages);
            sensorsEnumerator->getTemperatureSensors(temperatureSensors);
            sensors_are_created = true;
        }

        /* Add information about available sensors */

        // Depth sensor
        if (depthSensors.size() < 1) {
            buff_send.set_message("No depth sensors are available");
            buff_send.set_status(::payload::Status::UNREACHABLE);
            break;
        }
        camDepthSensor = depthSensors.front();
        aditof::SensorDetails depthSensorDetails;
        camDepthSensor->getDetails(depthSensorDetails);
        auto pbSensorsInfo = buff_send.mutable_sensors_info();
        sensorV4lBufAccess =
            std::dynamic_pointer_cast<aditof::V4lBufferAccessInterface>(
                camDepthSensor);

        // Storages
        int storage_id = 0;
        for (const auto &storage : storages) {
            std::string name;
            storage->getName(name);
            auto pbStorageInfo = pbSensorsInfo->add_storages();
            pbStorageInfo->set_name(name);
            pbStorageInfo->set_id(storage_id);
            ++storage_id;
        }

        // Temperature sensors
        int temp_sensor_id = 0;
        for (const auto &sensor : temperatureSensors) {
            std::string name;
            sensor->getName(name);
            auto pbTempSensorInfo = pbSensorsInfo->add_temp_sensors();
            pbTempSensorInfo->set_name(name);
            pbTempSensorInfo->set_id(temp_sensor_id);
            ++temp_sensor_id;
        }

        buff_send.set_status(
            static_cast<::payload::Status>(aditof::Status::OK));
        break;
    }

    case OPEN: {
        aditof::Status status = camDepthSensor->open();
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case START: {
        aditof::Status status = camDepthSensor->start();
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STOP: {
        aditof::Status status = camDepthSensor->stop();
        buff_send.set_status(static_cast<::payload::Status>(status));

        break;
    }

    case GET_AVAILABLE_FRAME_TYPES: {
        std::vector<aditof::FrameDetails> frameDetails;
        aditof::Status status =
            camDepthSensor->getAvailableFrameTypes(frameDetails);
        for (auto detail : frameDetails) {
            auto type = buff_send.add_available_frame_types();
            type->set_width(detail.width);
            type->set_height(detail.height);
            type->set_type(detail.type);
            type->set_full_data_width(detail.fullDataWidth);
            type->set_full_data_height(detail.fullDataHeight);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_FRAME_TYPE: {
        aditof::FrameDetails details;
        details.width = buff_recv.frame_type().width();
        details.height = buff_recv.frame_type().height();
        details.type = buff_recv.frame_type().type();
        details.fullDataWidth = buff_recv.frame_type().full_data_width();
        details.fullDataWidth = buff_recv.frame_type().full_data_height();
        aditof::Status status = camDepthSensor->setFrameType(details);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case PROGRAM: {
        size_t programSize = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint8_t *pdata = reinterpret_cast<const uint8_t *>(
            buff_recv.func_bytes_param(0).c_str());
        aditof::Status status = camDepthSensor->program(pdata, programSize);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_FRAME: {
        aditof::Status status = sensorV4lBufAccess->waitForBuffer();
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        struct v4l2_buffer buf;

        status = sensorV4lBufAccess->dequeueInternalBuffer(buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        unsigned int buf_data_len;
        uint8_t *buffer;

        status =
            sensorV4lBufAccess->getInternalBuffer(&buffer, buf_data_len, buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        buff_send.add_bytes_payload(buffer, buf_data_len * sizeof(uint8_t));

        status = sensorV4lBufAccess->enqueueInternalBuffer(buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        buff_send.set_status(payload::Status::OK);
        break;
    }

    case READ_AFE_REGISTERS: {
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(0).c_str());
        uint16_t *data = new uint16_t[length];
        aditof::Status status =
            camDepthSensor->readAfeRegisters(address, data, length);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(data, length * sizeof(uint16_t));
        }
        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case WRITE_AFE_REGISTERS: {
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(0).c_str());
        const uint16_t *data = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(1).c_str());
        aditof::Status status =
            camDepthSensor->writeAfeRegisters(address, data, length);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STORAGE_OPEN: {
        aditof::Status status;
        std::string msg;
        size_t index = static_cast<size_t>(buff_recv.func_int32_param(0));

        if (index < 0 || index >= storages.size()) {
            buff_send.set_message("The ID of the storage is invalid");
            buff_send.set_status(::payload::Status::INVALID_ARGUMENT);
            break;
        }

        void *sensorHandle;
        status = camDepthSensor->getHandle(&sensorHandle);
        if (status != aditof::Status::OK) {
            buff_send.set_message("Failed to obtain handle from depth sensor "
                                  "needed to open storage");
            buff_send.set_status(::payload::Status::GENERIC_ERROR);
            break;
        }

        status = storages[index]->open(sensorHandle);

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STORAGE_READ: {
        aditof::Status status;
        std::string msg;
        size_t index = static_cast<size_t>(buff_recv.func_int32_param(0));

        if (index < 0 || index >= storages.size()) {
            buff_send.set_message("The ID of the storage is invalid");
            buff_send.set_status(::payload::Status::INVALID_ARGUMENT);
            break;
        }

        uint32_t address = static_cast<uint32_t>(buff_recv.func_int32_param(1));
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(2));
        std::unique_ptr<uint8_t[]> buffer(new uint8_t[length]);
        status = storages[index]->read(address, buffer.get(), length);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(buffer.get(), length);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STORAGE_WRITE: {
        aditof::Status status;
        std::string msg;
        size_t index = static_cast<size_t>(buff_recv.func_int32_param(0));

        if (index < 0 || index >= storages.size()) {
            buff_send.set_message("The ID of the storage is invalid");
            buff_send.set_status(::payload::Status::INVALID_ARGUMENT);
            break;
        }

        uint32_t address = static_cast<uint32_t>(buff_recv.func_int32_param(1));
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(2));
        const uint8_t *buffer = reinterpret_cast<const uint8_t *>(
            buff_recv.func_bytes_param(0).c_str());

        status = storages[index]->write(address, buffer, length);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STORAGE_CLOSE: {
        aditof::Status status;
        std::string msg;
        size_t index = static_cast<size_t>(buff_recv.func_int32_param(0));

        if (index < 0 || index >= storages.size()) {
            buff_send.set_message("The ID of the storage is invalid");
            buff_send.set_status(::payload::Status::INVALID_ARGUMENT);
            break;
        }

        status = storages[index]->close();

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case TEMPERATURE_SENSOR_OPEN: {
        size_t index = static_cast<size_t>(buff_recv.func_int32_param(0));

        if (index < 0 || index >= temperatureSensors.size()) {
            buff_send.set_message(
                "The ID of the temperature sensor is invalid");
            buff_send.set_status(::payload::Status::INVALID_ARGUMENT);
            break;
        }

        void *sensorHandle;
        aditof::Status status = camDepthSensor->getHandle(&sensorHandle);
        if (status != aditof::Status::OK) {
            buff_send.set_message("Failed to obtain handle from depth sensor "
                                  "needed to open temperature sensor");
            buff_send.set_status(::payload::Status::GENERIC_ERROR);
            break;
        }

        status = temperatureSensors[index]->open(sensorHandle);

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case TEMPERATURE_SENSOR_READ: {
        size_t index = static_cast<size_t>(buff_recv.func_int32_param(0));

        if (index < 0 || index >= temperatureSensors.size()) {
            buff_send.set_message(
                "The ID of the temperature sensor is invalid");
            buff_send.set_status(::payload::Status::INVALID_ARGUMENT);
            break;
        }

        float temperature;
        aditof::Status status = temperatureSensors[index]->read(temperature);
        if (status == aditof::Status::OK) {
            buff_send.add_float_payload(temperature);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case TEMPERATURE_SENSOR_CLOSE: {
        size_t index = static_cast<size_t>(buff_recv.func_int32_param(0));

        if (index < 0 || index >= temperatureSensors.size()) {
            buff_send.set_message(
                "The ID of the temperature sensor is invalid");
            buff_send.set_status(::payload::Status::INVALID_ARGUMENT);
            break;
        }

        aditof::Status status = temperatureSensors[index]->close();

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case HANG_UP: {
        if (sensors_are_created) {
            cleanup_sensors();
        }

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
    s_map_api_Values["FindSensors"] = FIND_SENSORS;
    s_map_api_Values["Open"] = OPEN;
    s_map_api_Values["Start"] = START;
    s_map_api_Values["Stop"] = STOP;
    s_map_api_Values["GetAvailableFrameTypes"] = GET_AVAILABLE_FRAME_TYPES;
    s_map_api_Values["SetFrameType"] = SET_FRAME_TYPE;
    s_map_api_Values["Program"] = PROGRAM;
    s_map_api_Values["GetFrame"] = GET_FRAME;
    s_map_api_Values["ReadAfeRegisters"] = READ_AFE_REGISTERS;
    s_map_api_Values["WriteAfeRegisters"] = WRITE_AFE_REGISTERS;
    s_map_api_Values["StorageOpen"] = STORAGE_OPEN;
    s_map_api_Values["StorageRead"] = STORAGE_READ;
    s_map_api_Values["StorageWrite"] = STORAGE_WRITE;
    s_map_api_Values["StorageClose"] = STORAGE_CLOSE;
    s_map_api_Values["TemperatureSensorOpen"] = TEMPERATURE_SENSOR_OPEN;
    s_map_api_Values["TemperatureSensorRead"] = TEMPERATURE_SENSOR_READ;
    s_map_api_Values["TemperatureSensorClose"] = TEMPERATURE_SENSOR_CLOSE;
    s_map_api_Values["HangUp"] = HANG_UP;
}
