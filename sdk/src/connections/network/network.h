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
#include "buffer.pb.h"

#include <condition_variable>
#include <libwebsockets.h>
#include <thread>

class Network;

struct NetworkHandle {
    Network *net;
    std::mutex net_mutex;
};

class Network {

    static struct lws *web_socket;
    static struct lws_context *context;

    std::thread threadObj;
    static std::mutex m_mutex;
    static std::mutex mutex_recv;
    std::mutex thread_mutex;
    static std::condition_variable Cond_Var;
    static std::condition_variable thread_Cond_Var;

    static bool Send_Successful;
    static bool Data_Received;
    static bool Server_Connected;

    int Thread_Running;

    //! call_lws_service - calls lws_service api to service any websocket
    //! activity
    void call_lws_service();

  public:
    static payload::ClientRequest send_buff;
    static payload::ServerResponse recv_buff;

    //! ServerConnect() - APi to initialize the websocket and connect to
    //! websocket server
    int ServerConnect(const std::string &ip);

    //! SendCommand() - APi to send SDK apis to connected server
    int SendCommand();

    //! recv_server_data() - APi to receive data from server
    int recv_server_data();

    //! callback_function() - APi to handle websocket events
    static int callback_function(struct lws *wsi,
                                 enum lws_callback_reasons reason, void *user,
                                 void *in, size_t len);

    //! Network() - APi to initialize network parameters
    Network();

    //! ~Network() - destructor for network
    ~Network();

    //! isSend_Successful() - APi to check if data has been sent to server
    //! successfully
    bool isSend_Successful();

    //! isData_Received() - APi to check if data is received from server
    //! successfully
    bool isData_Received();

    //! isThread_Running() - APi to check thread exist or not
    bool isThread_Running();

    //! isServer_Connected() - APi to check if server is connected successfully
    bool isServer_Connected();
};
