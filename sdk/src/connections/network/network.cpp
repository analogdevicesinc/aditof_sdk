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
#include "network.h"

#include <functional>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <iostream>

#define RX_BUFFER_BYTES (4147900 * 2)
#define MAX_RETRY_CNT 3

enum protocols { PROTOCOL_0 = 0, PROTOCOL_COUNT };

using namespace google::protobuf::io;
using namespace payload;
using namespace std;

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

int nBytes = 0;          /*no of bytes sent*/
int recv_data_error = 0; /*flag for recv data*/
char server_msg[] = "Connection Allowed";

/*Declare static members*/
lws *Network::web_socket = NULL;
lws_context *Network::context = NULL;
ClientRequest Network::send_buff;
ServerResponse Network::recv_buff;
recursive_mutex Network::m_mutex;
mutex Network::mutex_recv;
condition_variable_any Network::Cond_Var;
condition_variable Network::thread_Cond_Var;

bool Network::Send_Successful;
bool Network::Data_Received;
bool Network::Server_Connected;

/*
* isServer_Connected(): checks if server is connected
* Parameters:        none
* returns:           true  - if server is connected
                     false - if no server is connected
* Desription:   This function checks if server is connected and returns
Server_Connected flag value.
*/
bool Network::isServer_Connected() { return Network::Server_Connected; }

/*
* isThread_Running(): check if thread created is running
* Parameters:        none
* returns:           true  - if thread is alive
                     false - if thread is not running
* Desription:   This function returns Thread_Running flag value to check if
thread is running.
*/
bool Network::isThread_Running() {
    /*Return true if thread has completed*/
    if (Network::Thread_Running == 2) {
        return true;
    } else {
        return false;
    }
}

/*
* isData_Received(): check if data is sent to server and returns Send_Successful
flag value
* Parameters:        none
* returns:           true  - if data has been sent succesfully
                     false - if error in data sending
* Desription:   This function returns Send_Successful flag value
*/
bool Network::isSend_Successful() { return Network::Send_Successful; }

/*
* isData_Received(): check if data received from server and returns
Data_Received flag value
* Parameters:        none
* returns:           true  - if data received successfully
                     false - if error in data receiving
* Desription:   This function is used to check if any data received from server
*               returns Data_Received flag value.
*/
bool Network::isData_Received() { return Network::Data_Received; }

/*
* ServerConnect():  intializes the websocket and connects to server
* Parameters:       ip - the ip address of the server to connect to
* returns:          0 - on success
                   -1 - on error
* Desription:   This function initializes the websocket and connects to server.
*/
int Network::ServerConnect(const std::string &ip) {
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));

    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;
    info.pt_serv_buf_size = 4096;

    /*Create a websocket for client*/
    this->context = lws_create_context(&info);

    struct lws_client_connect_info ccinfo = {0};
    ccinfo.context = this->context;
    ccinfo.address = ip.c_str();
    ccinfo.port = 5000;
    ccinfo.path = "/";
    ccinfo.host = lws_canonical_hostname(this->context);
    ccinfo.origin = "origin";
    ccinfo.protocol = protocols[PROTOCOL_0].name;

    web_socket = lws_client_connect_via_info(&ccinfo);

    /*Start a new thread to service any pending event on web socket*/

    threadObj = std::thread(&Network::call_lws_service, this);

    threadObj.detach();

    /*Wait for thread to be ready and server is connected*/

    std::unique_lock<std::recursive_mutex> mlock(m_mutex);

    /*Wait till server is connected or timeout of 3 sec*/
    if (Cond_Var.wait_for(mlock, std::chrono::seconds(3),
                          std::bind(&Network::isServer_Connected, this)) ==
        false) {
        Server_Connected = false;
        return -1;
    } else if (web_socket != NULL) {
        /*Wait for Server message to check another client is connected already
         * or not*/
        if (recv_server_data() == 0) {
            /*Data received correctly*/
            if (strcmp(recv_buff.message().c_str(), server_msg) == 0) {
                /*Server is connected successfully*/
                cout << "Conn established" << endl;
                return 0;
            } else {
                /*Another client is connected already*/
                cout << "Server Message :: " << recv_buff.message() << endl;
                Server_Connected = false;
                return -1;
            }
        } else {
            /*No message received from Server*/
            Server_Connected = false;
            return -1;
        }
    } else if (web_socket == NULL) {
        Server_Connected = false;
        return -1;
    }

    return -1;
}

/*
* sendCommand(): send data to server
* Parameters:    none
* returns:       0 - on success
                -1 -  on error
* Desription:    This function is used to send the data to connected server.
*/
int Network::SendCommand() {
    int status = -1;
    uint8_t numRetry = 0;
    int siz = send_buff.ByteSize();

    recv_buff.Clear();

    while (numRetry++ < MAX_RETRY_CNT && Server_Connected != false) {

        lws_callback_on_writable(web_socket);
        /*Acquire the lock*/
        std::unique_lock<std::recursive_mutex> mlock(m_mutex);

        if (Cond_Var.wait_for(mlock, std::chrono::seconds(5),
                              std::bind(&Network::isSend_Successful, this)) ==
            false) {
            status = -1; /*timeout occurs*/
#ifdef NW_DEBUG
            cout << "Send Timeout error" << endl;
#endif
            break;
        }

        Send_Successful = false;

        if (nBytes < 0) {
            /*Send failed Network issue*/
#ifdef NW_DEBUG
            cout << "Write Error" << endl;
#endif
            status = -1;
            break;
        } else if (nBytes < siz) {
            /*Retry sending command*/
#ifdef NW_DEBUG
            cout << "Retry sending " << numRetry << " times" << endl;
#endif
            status = -1;
        } else if (nBytes == siz) {
#ifdef NW_DEBUG
            cout << "Write Successful" << endl;
#endif
            status = 0;
            break;
        }
    }

    if (Server_Connected == false) {
        status = -2;
    }

    return status;
}

/*
* recv_server_data():  receive data from server
* Parameters:   None
* returns:      0  - on success
                -1 -  on error
* Desription:   This function is used to receive the data from connected server
*/
int Network::recv_server_data() {
    int status = -1;
    uint8_t numRetry = 0;
    /*Wait until data received from the server after command execution*/

    while (numRetry++ < MAX_RETRY_CNT && Server_Connected != false) {

        /*Acquire the lock*/
        std::unique_lock<std::mutex> mlock(mutex_recv);
        if (Cond_Var.wait_for(mlock, std::chrono::seconds(10),
                              std::bind(&Network::isData_Received, this)) ==
            true) {
            /*reset the flag value to receive again*/
            Data_Received = false;

            if (recv_data_error == 1) {
                /*Retry sending command*/
                if (SendCommand() != 0) {
                    status = -1;
                    break;
                }
            } else if (recv_data_error == 0) {
                /*Data received correctly*/
                status = 0;
                break;
            }

        } else {
            /*No data recvd till time out Retry sending command */
            if (SendCommand() != 0) {
                status = -1;
                break;
            }
        }
    }

    if (Server_Connected == false) {
        status = -2;
    }

    send_buff.Clear();

    return status;
}

/*
 * call_lws_service():  calls websockets library lws_service() api
 * Parameters:   None
 * returns:      None
 * Desription:   This function calls websockets library api to service any
 * pending websocket activity
 */
void Network::call_lws_service() {
    while (1) {
        lws_service(this->context, 0);
#ifdef NW_DEBUG
        cout << ".";
#endif
        /*Complete the thread if destructor is called*/
        std::lock_guard<std::mutex> guard(thread_mutex);

        if (Thread_Running == 1) {
#ifdef NW_DEBUG
            cout << "Thread exited" << endl;
#endif
            Thread_Running = 2;
            thread_Cond_Var.notify_one();
            break;
        }
    }
}

/*
* callback_function():  Handles the websocket events
* Parameters:           wsi - websocket instance pointer
                        reasons - websocket event occurred for wsi instance
                        user - pointer to per session user data allocated by
libwebsocket in   - pointer to data len  - length of data
* returns:              0
* Desription:           This function handles the websocket events and take
appropriate action
*/
int Network::callback_function(struct lws *wsi,
                               enum lws_callback_reasons reason, void *user,
                               void *in, size_t len) {
    switch (reason) {
    case LWS_CALLBACK_CLIENT_ESTABLISHED: {
        /*Notify host SDK that server is connected */
        std::lock_guard<std::recursive_mutex> guard(m_mutex);
        Server_Connected = true;
        Cond_Var.notify_one();
        break;
    }

    case LWS_CALLBACK_CLIENT_RECEIVE: {
        /* Handle incoming messages here. */
#ifdef NW_DEBUG
        cout << endl << "Rcvd Data len : " << len << endl;
#endif
        std::lock_guard<std::mutex> guard(mutex_recv);

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
            google::protobuf::io::ArrayInputStream ais(in,
                                                       static_cast<int>(len));
            CodedInputStream coded_input(&ais);
            recv_buff.ParseFromCodedStream(&coded_input);

            recv_data_error = 0;
            Data_Received = true;

            /*Notify the host SDK that data is received from server*/
            Cond_Var.notify_one();

        } else {
            // append message
            if (clientData->data.size() == 0) {
                clientData->data.reserve(len + remaining);
            }

            std::cout << "apending data" << std::endl;
            char *inData = static_cast<char *>(in);
            clientData->data.insert(clientData->data.end(), inData,
                                    inData + len);
            clientData->hasFragments = true;
        }

        break;
    }

    case LWS_CALLBACK_CLIENT_WRITEABLE: {
#ifdef NW_DEBUG
        cout << endl << "Client is sending " << send_buff.func_name() << endl;
#endif
        std::lock_guard<std::recursive_mutex> guard(m_mutex);
        if (send_buff.func_name().empty()) {
            break;
        }

        /* Get size of packet to be sent*/
        int siz = send_buff.ByteSize();
        /*Pre padding of bytes as per websockets*/
        unsigned char *pkt =
            new unsigned char[siz + LWS_SEND_BUFFER_PRE_PADDING];
        unsigned char *pkt_pad = pkt + LWS_SEND_BUFFER_PRE_PADDING;

        google::protobuf::io::ArrayOutputStream aos(pkt_pad, siz);
        CodedOutputStream *coded_output = new CodedOutputStream(&aos);
        send_buff.SerializeToCodedStream(coded_output);

        nBytes = lws_write(wsi, pkt_pad, siz, LWS_WRITE_TEXT);

        /*Notify the host SDK that data is sent to server*/
        Send_Successful = true;
        Cond_Var.notify_one();

        delete coded_output;
        delete[] pkt;
        send_buff.Clear();
        break;
    }

    case LWS_CALLBACK_CLIENT_CLOSED: {
        cout << "Connection Closed" << endl;
        /*Set a flag to indicate server connection is closed abruptly*/
        std::lock_guard<std::recursive_mutex> guard(m_mutex);
        Server_Connected = false;
        web_socket = NULL;
        break;
    }

    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR: {
        cout << "Connection Error" << endl;
        web_socket = NULL;
        break;
    }

    default:
        break;
    }

    return 0;
}

/*
 * Network():    Initializes the network parameters
 * Parameters:   None
 * Desription:   This function initializes the network parameters
 */
Network::Network() {
    this->context = NULL;

    /*Initialize the static flags*/
    Network::Send_Successful = false;
    Network::Data_Received = false;
    Network::Thread_Running = 0;
    Network::Server_Connected = false;
}

/*
 * ~Network():   Destructor for network class
 * Parameters:   None
 * Desription:   Destructor for network class
 */
Network::~Network() {
    if (this->context != NULL) {
        /*set a flag to complete the thread */
        std::unique_lock<std::mutex> mlock(thread_mutex);
        Thread_Running = 1;
        /*wait for thread to finish*/
        thread_Cond_Var.wait(mlock,
                             std::bind(&Network::isThread_Running, this));

        lws_context_destroy(this->context);
    }
}
