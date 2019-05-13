/********************************************************************************/
/*                                                                              */
/* @file        Server.h                                                        */
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
#include <libwebsockets.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <map>
#include "buffer.pb.h"
#include <iostream>
#include "types.h"
#include "ADI_TOF.h"
#include <string>

#define RX_BUFFER_BYTES (1240000)
#define INTRINSIC_TBL_SIZE 9
#define DISTORTION_TBL_SIZE 5

class Network
{
public:
  struct lws_context * context;
  Network();
  // ~Network();
  static int callback_function(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len);
};

enum protocols
{
  PROTOCOL_EXAMPLE,
  PROTOCOL_COUNT
};

enum api_Values
{
  API_NOT_DEFINED,
  INIT,
  PROC_FRAME,
  RAW_PROC_FRAME,
  PRE_PROC_FRAME,
  DEPTH_PROC_FRAME,
  POST_PROC_FRAME,
  RELEASE,
  SET_FILTER,
  SET_NR_FILTER,
  SET_THRESHOLD,
  SET_DEPTH_RANGE,
  GET_DEPTH_RANGE,
  GET_DEPTH_MAX,
  SET_CANCELLATION,
  ACTIVATE_CANCELLATION,
  GET_CANCELLATION_STATUS,
  SET_MULTI_RANGE,
  GET_MULTI_RANGE,
  GET_CAMERA_INTRINSIC,
  SET_RADIAL_CORRECTION,
  SET_PULSE_COUNT,
  GET_PULSE_COUNT,
  SET_TAL_ENABLE,
  GET_TAL_ENABLE,
  SET_LASER_RIGHT,
  SET_LASER_LEFT,
  GET_ERROR_STRING,
  SET_LOG_CONFIG,
  PROGRAM_AFE,
  GET_DEPTH_DATA,
  GET_IR_DATA,
  GET_IMAGE,
  GET_S0_DATA,
  GET_S1_DATA
};

