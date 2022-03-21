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
#ifndef __MWDEMOIMAQ_HEADER__
#define __MWDEMOIMAQ_HEADER__

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

#include <iostream>

#include <mwadaptorimaq.h>
#ifdef _WIN32
#include <windows.h>
#else
#endif

namespace aditof {

// ********** Demo Adaptor Information **********
/// Return string for DemoAdaptor::getDriverDescription().
const char *const DRIVER_DESCRIPTION_STR = "ADI AD-96TOF1-EBZ Device Driver";

/// Return string for DemoAdaptor::getDriverVersion().
const char *const DRIVER_VERSION_STR = "Version 0.1 (Beta 1)";

// ********** Time of Flight Device Information **********

const char *const USB_DEVICE_STR = "ToF Usb Device";
const char *const NETWORK_DEVICE_STR = "ToF Network Device";

enum { USB_DEVICE_ID = 1, NETWORK_DEVICE_ID = 2 };

// end aditof

/// Default width if no image format is found.
const int DEMO_DEFAULT_WIDTH = 640;
const int DEMO_DEFAULT_HEIGHT = 480;
const int DEMO_DEFAULT_NUMBER_OF_BANDS = 3;

// ********** Color Device Information **********
/// RGB Format Information
const char *const BGR_FORMAT_STR = "BGR";
const int BGR_FORMAT_ID = 0;
const int BGR_FORMAT_WIDTH = 640;
const int BGR_FORMAT_HEIGHT = 480;
const int BGR_FORMAT_BANDS = 1;

const char *const MONO_FORMAT_STR = "MONO";
const int MONO_FORMAT_ID = 1;
const int MONO_FORMAT_WIDTH = 640;
const int MONO_FORMAT_HEIGHT = 480;
const int MONO_FORMAT_BANDS = 1;

const char *const MONO16_FORMAT_STR = "MONO16";
const int MONO16_FORMAT_ID = 2;
const int MONO16_FORMAT_WIDTH = 640;
const int MONO16_FORMAT_HEIGHT = 480;
const int MONO16_FORMAT_BANDS = 1;

const char *const RGB_CAMERA_FORMAT_STR = "RGB_CAMERA";
const int RGB_CAMERA_FORMAT_ID = 3;
const int RGB_CAMERA_FORMAT_WIDTH = 1920;
const int RGB_CAMERA_FORMAT_HEIGHT = 1080;
const int RGB_CAMERA_FORMAT_BANDS = 1;

enum {
    ADITOF_PROPERTY_MODE = 1,
    ADITOF_PROPERTY_FRAME_TYPE = 2,
    ADITOF_PROPERTY_NOISE_REDUCTION_THRESHOLD = 3,
    ADITOF_PROPERTY_TEMP = 4,
    ADITOF_PROPERTY_IR_GAMMA_CORRECTION = 5,
    ADITOF_PROPERTY_DEPTH_CORRECTION = 6,
    ADITOF_PROPERTY_GEOMETRY_CORRECTION = 7
};

/// Mode enum property
const char *const MODE_STR = "CameraMode";
const char *const MODE_NEAR_STR = "Near";
const char *const MODE_MEDIUM_STR = "Medium";
const char *const MODE_FAR_STR = "Far";

enum { MODE_NEAR_ID = 1, MODE_MEDIUM_ID = 2, MODE_FAR_ID = 3 };

const char *const FRAME_TYPE_STR = "FrameType";
const char *const FRAME_TYPE_DEPTH_RGB_STR = "DepthRgb";
const char *const FRAME_TYPE_IR_RGB_STR = "IrRgb";
const char *const FRAME_TYPE_DEPTH_RAW_STR = "DepthRaw";
const char *const FRAME_TYPE_IR_RAW_STR = "IrRaw";
const char *const FRAME_TYPE_DEPTH_IR_RAW_STR = "DepthIrRaw";
const char *const FRAME_TYPE_RGB_STR = "Rgb";

enum {
    FRAME_TYPE_DEPTH_RGB_ID = 1,
    FRAME_TYPE_IR_RGB_ID = 2,
    FRAME_TYPE_DEPTH_RAW_ID = 3,
    FRAME_TYPE_IR_RAW_ID = 4,
    FRAME_TYPE_DEPTH_IR_RAW_ID = 5,
    FRAME_TYPE_RGB_ID = 6
};

const char *const NOISE_REDUCTION_THRESHOLD_STR = "NoiseReductionThreshold";
const int64_t NOISE_REDUCTION_THRESHOLD_LOWER_LIMIT = 0;
const int64_t NOISE_REDUCTION_THRESHOLD_UPPER_LIMIT = 16383;
const int64_t NOISE_REDUCTION_THRESHOLD_DEFAULT = 50;

const char *const IR_GAMMA_CORRECTION_STR = "IrGammaCorrection";
const char *const IR_GAMMA_CORRECTION_DEFAULT = "1";

const char *const DEPTH_CORRECTION_STR = "DepthCorrection";
const int64_t DEPTH_CORRECTION_LOWER_LIMIT = 0;
const int64_t DEPTH_CORRECTION_UPPER_LIMIT = 1;
const int64_t DEPTH_CORRECTION_DEFAULT = 1;

const char *const GEOMETRY_CORRECTION_STR = "GeometryCorrection";
const int64_t GEOMETRY_CORRECTION_LOWER_LIMIT = 0;
const int64_t GEOMETRY_CORRECTION_UPPER_LIMIT = 1;
const int64_t GEOMETRY_CORRECTION_DEFAULT = 1;

const char *const TEMPERATURE_STR = "Temperature";

/// Properties with custom get functions
const char *const TIMESTAMP_STRING_STR = "TimestampString";
const char *const TIMESTAMP_INT_STR = "TimestampInteger";

/// Monochrome Device Source Information
const char *const SOURCE_INPUT_1_STR = "input1";
const char *const SOURCE_INPUT_2_STR = "input2";
const char *const SOURCE_INPUT_3_STR = "input3";
const char *const SOURCE_INPUT_4_STR = "input4";
const char *const SOURCE_INPUT_5_STR = "input5";
const char *const SOURCE_INPUT_6_STR = "input6";

enum {
    SOURCE_INPUT_1_ID = 1,
    SOURCE_INPUT_2_ID = 2,
    SOURCE_INPUT_3_ID = 3,
    SOURCE_INPUT_4_ID = 4,
    SOURCE_INPUT_5_ID = 5,
    SOURCE_INPUT_6_ID = 6
};

// ********** Digital Hardware Information **********/// Property to be added
// from XML section.
const char *const DIGINPUT_STR = "digInput";

// ********** Other information **********
/// Generated image data offset value.
/// The higher the number the faster the shifting from right to left looks.
const int IMAGE_DATA_OFFSET = 2;

/// This is the value in milliseconds that the image generator pauses to
/// make the frame rate more realistic.
const int IMAGE_FRAME_PAUSE_TIME = 30;

#ifdef _WIN32
/// Time, in milliseconds, given for an object to finish.
/// Used when calling WaitForSingleObject() in DemoAdaptor::closeDevice().
const DWORD SINGLE_OBJECT_WAIT_TIME = 10000;
#endif

// ********** ERROR IDs and MESSAGES **********
/// Error ID and Message when the device is not found in getDeviceAttributes().
const char *const ERRID_DEVICE_NOT_FOUND = "imaq:demo:devicenotfound";
const char *const ERRMSG_DEVICE_NOT_FOUND =
    "Unable to find the image acquisition device.";

/// To be removed when a routine for displaying non-warning messages in MATLAB
/// is implemented.
const char *const WARNID_CONFIGURING_PROPERTY = "imaq:demo:configuringproperty";
} // namespace aditof

#endif
