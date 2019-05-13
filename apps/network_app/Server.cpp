/********************************************************************************/
/*                                                                              */
/* @file        Server.cpp                                                      */
/*                                                                              */
/* @brief       network server related APIs                                     */
/*                                                                              */
/* @author      Harshada Sarnaik                                                */
/*                                                                              */
/* @date                                                                        */
/*                                                                              */
/* Copyright(c) Analog Devices, Inc.                                            */
/*                                                                              */
/********************************************************************************/

#include "Server.h"
#include <sys/time.h>
#include "chrono"
#define IMAGE_SIZE (642 * 482)
#define RAW_IMAGE_SIZE (640 * 480 * 2)

using namespace google::protobuf::io;

using namespace std;

struct timeval start, finish;
int msec = 0;
unsigned int tCnt = 0;

ADI_TOF *adi_tof = NULL;
payload::host_buffer buff_recv;
payload::target_buffer buff_send;
static std::map<string, api_Values> s_map_api_Values;
static void Initialize();
void invoke_sdk_api(payload::host_buffer buff_recv);
bool Client_Connected = false;
bool no_of_client_connected = false;
void clear_send_buff();
void clear_recv_buff();

static struct lws_protocols protocols[] =
{
  {
    "network-protocol",
    Network::callback_function,
    0,
    RX_BUFFER_BYTES,
  },
  { NULL, NULL, 0, 0 } /* terminator */
};

Network :: Network()
{
  context = NULL;
}

int Network::callback_function( struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len )
{
  int n;

  switch( reason )
  {
    case LWS_CALLBACK_ESTABLISHED:
    {
      /*Check if another client is connected or not*/
      clear_send_buff();
      if(Client_Connected == false)
      {
        cout<<"Conn Established"<<endl;
        Client_Connected = true;
        adi_tof = new ADI_TOF();
        buff_send.set_message("Connection Allowed");
        lws_callback_on_writable(wsi);
        break;
      }
      else
      {
        cout<<"Another client connected"<<endl;
        no_of_client_connected = true;
        buff_send.set_message("Only 1 client connection allowed");
        lws_callback_on_writable(wsi);
      }
      break;
    }

    case LWS_CALLBACK_RECEIVE:
    {
#ifdef NW_DEBUG
      cout<< endl << "Server has received data with len: "<<len<<endl;
#endif
      google::protobuf::io::ArrayInputStream ais(in,len);
      CodedInputStream coded_input(&ais);
      buff_recv.ParseFromCodedStream(&coded_input);

      if(len < buff_recv.buff_size())
      {
#ifdef NW_DEBUG
        cout << "Partial data received....." << endl;
#endif
      }
      else
      {
        invoke_sdk_api(buff_recv);
        lws_callback_on_writable(wsi);
      }
      break;
    }

    case LWS_CALLBACK_SERVER_WRITEABLE:
    {
      int siz = buff_send.ByteSize();
      buff_send.set_buff_size(siz);
      siz = buff_send.ByteSize();
      buff_send.set_buff_size(siz);
      unsigned char *pkt = new unsigned char [siz+LWS_SEND_BUFFER_PRE_PADDING];
      unsigned char *pkt_pad = pkt+LWS_SEND_BUFFER_PRE_PADDING;
      google::protobuf::io::ArrayOutputStream aos(pkt_pad,siz);
      CodedOutputStream *coded_output = new CodedOutputStream(&aos);
      buff_send.SerializeToCodedStream(coded_output);

      n = lws_write( wsi, pkt_pad, (siz), LWS_WRITE_TEXT );
#ifdef NW_DEBUG
      cout << "server is sending " << n << endl;
#endif
      if(n < 0)
        cout<<"Error Sending"<<endl;
      else if(n < siz)
        cout<<"Partial write"<<endl;
      else if(n == siz)
      {
#ifdef NW_DEBUG
        cout<<"Write successful"<<endl;
#endif
      }
      delete coded_output;
      delete[] pkt; 
      break;
    }

    case LWS_CALLBACK_CLOSED:
    {
      if(Client_Connected == true && no_of_client_connected == false)
      {
        /*CONN_CLOSED event is for first and only client connected*/
        cout << "Connection Closed" << endl;
        delete adi_tof;
        adi_tof = NULL;
        Client_Connected = false;
        break;
      }
      else
      {
        /*CONN_CLOSED event for more than 1 client connected */
        cout << "Another Client Connection Closed" << endl;
        no_of_client_connected = false;
        break;
      }
    }

    default:
    {
#ifdef NW_DEBUG
      cout<<reason<<endl;
#endif
    }
      break;
  }

  return 0;
}


int main( int argc, char *argv[] )
{
  struct lws_context_creation_info info;
  memset( &info, 0, sizeof(info) );

  info.port = 5000;
  info.protocols = protocols;
  info.gid = -1;
  info.uid = -1;
  Network *network = new Network();

  network->context = lws_create_context( &info );

  Initialize();

#if 0
  /* Note: Simply enabling this won't work, need libwebsocket compiled differently to demonize this */
  if(lws_daemonize("/tmp/server_lock"))
  {
    fprintf(stderr,"Failed to daemonize\n");
  }
#endif

  while( 1 )
  {
    lws_service( network->context, 0 /* timeout_ms */);
  }

  lws_context_destroy( network->context );
  delete adi_tof;
  delete network;
  return 0;
}

void invoke_sdk_api(payload::host_buffer buff_recv)
{
  string func_name = buff_recv.func_name();
  /*clear send buffer*/
  clear_send_buff();

  switch(s_map_api_Values[func_name])
  {
    case INIT:
    {
#ifdef DEBUG
      cout<< "Init function"<< endl;
#endif
      tCnt = 0;

      gettimeofday (&start, NULL);
      bool playBackEna = ((buff_recv.func_param(0) == 1)? true : false);
      int ret_val;
      ret_val = adi_tof->Init(playBackEna,buff_recv.func_param(1));
      if(ret_val < 0)
      {
        buff_send.set_api_successful(false);
        buff_send.set_ret_val(ret_val);
      }
      else
      {
        buff_send.set_api_successful(true);
      }
      break;
    }

    case PROC_FRAME:
    {
#ifdef DEBUG
      cout << "ProcFrame function" << endl;
#endif
      uint16_t *punRawIn = NULL;
      gettimeofday (&start, NULL);     /// Either this 
      adi_tof->ProcFrame(punRawIn);
      gettimeofday (&finish, NULL);

      msec = (finish.tv_sec*1000 + finish.tv_usec/1000);
      msec -= (start.tv_sec*1000 + start.tv_usec/1000);
      printf ("\n %d, ProcFrame: %d ms", tCnt, msec);
      tCnt++;
      //gettimeofday (&start, NULL);     /// Or this 

      buff_send.add_ret_data(adi_tof->width);
      buff_send.add_ret_data(adi_tof->height);
#if INT_LOGIC
      for(int i=0; i< IMAGE_SIZE; i++)
      {
        buff_send.add_ret_data(*(adi_tof->punDepth));
        adi_tof->punDepth++;
      }
#endif
      buff_send.set_image_data((char *)adi_tof->punDepth, IMAGE_SIZE*2);
      buff_send.set_ir_image_data((char *)adi_tof->punIR, IMAGE_SIZE*2);
      buff_send.set_api_successful(true);
      break;
    }

    case RELEASE:
    {
#ifdef DEBUG
      cout << "Release function" << endl;
#endif
      adi_tof->Release();
      buff_send.set_api_successful(true);
      break;
    }

    case SET_FILTER:
    {
#ifdef DEBUG
      cout << "setFilter function" << endl;
#endif
      adi_tof->setFilter((eFilterSel_t)buff_recv.func_param(0));
      buff_send.set_api_successful(true);
      break;
    }

    case SET_NR_FILTER:
    {
#ifdef DEBUG
      cout << "setNRFilter function" << endl;
#endif
      adi_tof->setNRFilter((bool)buff_recv.func_param(0));
      buff_send.set_api_successful(true);
      break;
    }

    case SET_THRESHOLD:
    {
#ifdef DEBUG
      cout << "setThreshold function" << endl;
#endif
      adi_tof->setThreshold((uint16)buff_recv.func_param(0));
      buff_send.set_api_successful(true);
      break;
    }

    case SET_DEPTH_RANGE:
    {
#ifdef DEBUG
      cout << "setDepthRange function" << endl;
#endif
      adi_tof->setDepthRange(range(buff_recv.func_param(0)),uint16(buff_recv.func_param(1)));
      buff_send.set_api_successful(true);
      break;
    }

    case GET_DEPTH_RANGE:
    {
#ifdef DEBUG
      cout << "getDepthRange function" << endl;
#endif
      buff_send.set_ret_val(adi_tof->getDepthRange());
      buff_send.set_api_successful(true);
      break;
    }

    case GET_DEPTH_MAX:
    {
#ifdef DEBUG
      cout << "getDepthMax function" << endl;
#endif
      buff_send.set_ret_val(adi_tof->getDepthMax());
      buff_send.set_api_successful(true);
      break;
    }

    case SET_CANCELLATION:
    {
#ifdef DEBUG
      cout << "setCancellation function" << endl;
#endif
      adi_tof->setCancellation(cancellationPattern(buff_recv.func_param(0)));
      buff_send.set_api_successful(1);
      break;
    }

    case ACTIVATE_CANCELLATION:
    {
#ifdef DEBUG
      cout << "activateCancellation function" << endl;
#endif
      adi_tof->activateCancellation((bool)buff_recv.func_param(0));
      buff_send.set_api_successful(1);
      break;
    }

    case GET_CANCELLATION_STATUS:
    {
#ifdef DEBUG
      cout << "getCancellationStatus function" << endl;
#endif
      buff_send.set_ret_val(adi_tof->getCancellationStatus());
      break;
    }

    case SET_MULTI_RANGE:
    {
#ifdef DEBUG
      cout << "setMultiRange function" << endl;
#endif
      adi_tof->setMultiRange((bool)buff_recv.func_param(0));
      buff_send.set_api_successful(1);
      break;
    }

    case GET_MULTI_RANGE:
    {
#ifdef DEBUG
      cout << "getMultiRange function" << endl;
#endif
      // adi_tof->getMultiRange();
      break;
    }

    case GET_CAMERA_INTRINSIC:
    {
#ifdef DEBUG
      cout << "getCameraIntrinsic function" << endl;
#endif
      float *pfIntrinsics =NULL;
      double *pDistortionCoeff = NULL;
      adi_tof->getCameraIntrinsic(&pfIntrinsics,&pDistortionCoeff);
      for(int i=0; i< INTRINSIC_TBL_SIZE; i++)
        buff_send.add_fintrinsic(*pfIntrinsics++);
      for(int i = 0; i< DISTORTION_TBL_SIZE; i++)
        buff_send.add_ddistortioncoeff(*pDistortionCoeff++);

      buff_send.set_api_successful(true);
      break;
    }

    case SET_RADIAL_CORRECTION:
    {
#ifdef DEBUG
      cout << "setRadialCorrection function" << endl;
#endif
      adi_tof->setRadialCorrection(bool(buff_recv.func_param(0)));
      buff_send.set_api_successful(1);
      break;
    }

    case SET_PULSE_COUNT:
    {
#ifdef DEBUG
      cout << "setPulseCount function" << endl;
#endif
      adi_tof->setPulseCount(uint16(buff_recv.func_param(0)));
      buff_send.set_api_successful(true);
      break;
    }

    case GET_PULSE_COUNT:
    {
#ifdef DEBUG
      cout << "getPulseCount function" << endl;
#endif
      buff_send.set_ret_val(adi_tof->getPulseCount());
      buff_send.set_api_successful(true);
      break;
    }

    case SET_TAL_ENABLE:
    {
#ifdef DEBUG
      cout << "setTALEnable function" << endl;
#endif
      adi_tof->setTALEnable(bool(buff_recv.func_param(0)));
      buff_send.set_api_successful(1);
      break;
    }

    case GET_TAL_ENABLE:
    {
#ifdef DEBUG
      cout << "getTALEnable function" << endl;
#endif
      buff_send.set_ret_val(adi_tof->getTALEnable());
      break;
    }

    case SET_LASER_RIGHT:
    {
#ifdef DEBUG
      cout << "setLaserRight function" << endl;
#endif
      adi_tof->setLaserRight((bool)buff_recv.func_param(0));
      buff_send.set_api_successful(1);
      break;
    }

    case SET_LASER_LEFT:
    {
#ifdef DEBUG
      cout << "setLaserLeft function " << endl;
#endif
      adi_tof->setLaserLeft((bool)buff_recv.func_param(0));
      buff_send.set_api_successful(1);
      break;
    }

    case GET_ERROR_STRING:
    {
#ifdef DEBUG
      cout << "getErrorString function" << endl;
#endif
      // adi_tof->getErrorString(buff_recv.func_param(0));
      // need to update the message buffer as this function returns string
      break;
    }

    case SET_LOG_CONFIG:
    {
#ifdef DEBUG
      cout << "setLogConfig function" << endl;
#endif
      adi_tof->setLogConfig((bool)buff_recv.func_param(0),(bool)buff_recv.func_param(1));
      buff_send.set_api_successful(1);
      break;
    }

    case PROGRAM_AFE:
    {
#ifdef DEBUG
      cout << "programAFE function" << endl;
#endif
      // adi_tof->programAFE();
      break;
    }

    case GET_DEPTH_DATA:
    {
      buff_send.add_ret_data(adi_tof->width);
      buff_send.add_ret_data(adi_tof->height);
      for(int i=0; i< IMAGE_SIZE; i++)
      {
        buff_send.add_ret_data(*(adi_tof->punDepth));
        adi_tof->punDepth++;
      }
      break;
    }

    case GET_IR_DATA:
    {
#ifdef DEBUG
      cout << "IR function" << endl;
#endif
#if INT_LOGIC
      for(int i=0;i<IMAGE_SIZE;i++)
      {
        buff_send.add_ret_data(*(adi_tof->punIR));
        adi_tof->punIR++;
      }
#endif
      buff_send.set_image_data((char *)adi_tof->punIR,IMAGE_SIZE*2);
      buff_send.set_api_successful(true);
      break;
    }

    case GET_IMAGE:
    {
      /*Call getImage() API to get raw image data from v4l2 driver*/
#ifdef DEBUG
      cout<<"getImage"<<endl;
#endif
      uint16 *punRaw_Image;
      punRaw_Image = adi_tof->getImage();
      buff_send.set_image_data((char *)punRaw_Image, (RAW_IMAGE_SIZE*1.5));
      buff_send.set_api_successful(true);
      free(punRaw_Image);
      punRaw_Image = NULL;
      break;
    }

    case RAW_PROC_FRAME:
    {
#ifdef DEBUG
      cout << "rawProcFrame function" << endl;
#endif
      uint16_t *punRawIn = NULL;
      int ret_val;
      ret_val = adi_tof->rawProcFrame(punRawIn);
      if(ret_val < 0)
      {
        buff_send.set_api_successful(false);
        buff_send.set_ret_val(ret_val);
      }
      else
      {
        buff_send.add_ret_data(adi_tof->width);
        buff_send.add_ret_data(adi_tof->height);
        buff_send.set_image_data((char *)adi_tof->punDepth, IMAGE_SIZE*2);
        buff_send.set_api_successful(true);
      }
      break;
    }

    case PRE_PROC_FRAME:
    {
#ifdef DEBUG
      cout << "preProcFrame function" << endl;
#endif
      uint16_t *punRawIn = NULL;
      int ret_val;
      ret_val = adi_tof->preProcFrame(punRawIn);
      if(ret_val < 0)
      {
        buff_send.set_api_successful(false);
        buff_send.set_ret_val(ret_val);
      }
      else
      {
        buff_send.set_image_data((char *)adi_tof->punS2, IMAGE_SIZE*2);
        buff_send.set_api_successful(true);
      }
      break;
    }

    case DEPTH_PROC_FRAME:
    {
#ifdef DEBUG
      cout << "depthProcFrame function" << endl;
#endif
      int ret_val;
      ret_val = adi_tof->depthProcFrame();
      if(ret_val < 0)
      {
        buff_send.set_api_successful(false);
        buff_send.set_ret_val(ret_val);
      }
      else
      {
        buff_send.set_image_data((char *)adi_tof->punDepth, IMAGE_SIZE*2);
        buff_send.set_api_successful(true);
      }
      break;
    }

    case POST_PROC_FRAME:
    {
#ifdef DEBUG
      cout << "postProcFrame function" << endl;
#endif
      int ret_val;
      ret_val = adi_tof->postProcFrame();
      if(ret_val < 0)
      {
        buff_send.set_api_successful(false);
        buff_send.set_ret_val(ret_val);
      }
      else
      {
        buff_send.set_image_data((char *)adi_tof->punDepth, IMAGE_SIZE*2);
        buff_send.set_api_successful(true);
      }
      break;
    }

    case GET_S0_DATA:
    {
#ifdef DEBUG
      cout << "S0 function" << endl;
#endif
      buff_send.set_image_data((char *)adi_tof->punS0,IMAGE_SIZE*2);
      buff_send.set_api_successful(true);
      break;
    }

    case GET_S1_DATA:
    {
#ifdef DEBUG
      cout << "S1 function" << endl;
#endif
      buff_send.set_image_data((char *)adi_tof->punS1,IMAGE_SIZE*2);
      buff_send.set_api_successful(true);
      break;
    }

    default:
    {
      cout << "Function not found" << endl;
      break;
    }
  }

/*  clear recv buffer*/
  clear_recv_buff();
  return;
}

void Initialize()
{
  s_map_api_Values["Init"] = INIT;
  s_map_api_Values["ProcFrame"] = PROC_FRAME;
  s_map_api_Values["rawProcFrame"] = RAW_PROC_FRAME;
  s_map_api_Values["preProcFrame"] = PRE_PROC_FRAME;
  s_map_api_Values["depthProcFrame"] = DEPTH_PROC_FRAME;
  s_map_api_Values["postProcFrame"] = POST_PROC_FRAME;
  s_map_api_Values["Release"] = RELEASE;
  s_map_api_Values["setFilter"] = SET_FILTER;
  s_map_api_Values["setNRFilter"] = SET_NR_FILTER;
  s_map_api_Values["setThreshold"] = SET_THRESHOLD,
  s_map_api_Values["setDepthRange"] = SET_DEPTH_RANGE;
  s_map_api_Values["getDepthRange"] = GET_DEPTH_RANGE;
  s_map_api_Values["getDepthMax"] = GET_DEPTH_MAX;
  s_map_api_Values["setCancellation"] = SET_CANCELLATION;
  s_map_api_Values["activateCancellation"] = ACTIVATE_CANCELLATION;
  s_map_api_Values["getCancellationStatus"] = GET_CANCELLATION_STATUS;
  s_map_api_Values["setMultiRange"] = SET_MULTI_RANGE;
  s_map_api_Values["getMultiRange"] = GET_MULTI_RANGE ;
  s_map_api_Values["getCameraIntrinsic"] = GET_CAMERA_INTRINSIC ;
  s_map_api_Values["setRadialCorrection"] =  SET_RADIAL_CORRECTION;
  s_map_api_Values["setPulseCount"] =  SET_PULSE_COUNT;
  s_map_api_Values["getPulseCount"] =  GET_PULSE_COUNT;
  s_map_api_Values["setTALEnable"] =  SET_TAL_ENABLE;
  s_map_api_Values["getTALEnable"] =  GET_TAL_ENABLE;
  s_map_api_Values["setLaserRight"] = SET_LASER_RIGHT;
  s_map_api_Values["setLaserLeft"] = SET_LASER_LEFT ;
  s_map_api_Values["getErrorString"] = GET_ERROR_STRING;
  s_map_api_Values["setLogConfig"] = SET_LOG_CONFIG;
  s_map_api_Values["programAFE"] = PROGRAM_AFE;
  s_map_api_Values["Get_punDepth_Data"] = GET_DEPTH_DATA;
  s_map_api_Values["Get_punIR_Data"] = GET_IR_DATA;
  s_map_api_Values["getImage"] = GET_IMAGE;
  s_map_api_Values["Get_punS0_Data"] = GET_S0_DATA;
  s_map_api_Values["Get_punS1_Data"] = GET_S1_DATA;
}

void clear_send_buff()
{
  buff_send.clear_ret_data();
  buff_send.clear_ret_val();
  buff_send.clear_buff_size();
  buff_send.clear_api_successful();
  buff_send.clear_fintrinsic();
  buff_send.clear_ddistortioncoeff();
  buff_send.clear_image_data();
  buff_send.clear_ir_image_data();
  buff_send.clear_message();
}

void clear_recv_buff()
{
  buff_recv.clear_func_name();
  buff_recv.clear_func_param();
  buff_recv.clear_ptr_param_cnt();
  buff_recv.clear_has_ret();
  buff_recv.clear_buff_size();
}
