/********************************************************************************/
/* @file        Network.h */
/*                                                                              */
/* @brief       Header file for network related code */
/*                                                                              */
/* @author      Nikhil Karale */
/*                                                                              */
/* @date        November 21, 2018 */
/*                                                                              */
/* Copyright(c) Analog Devices, Inc. */
/*                                                                              */
/********************************************************************************/

#include "buffer.pb.h"

#include <condition_variable>
#include <libwebsockets.h>
#include <thread>

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
