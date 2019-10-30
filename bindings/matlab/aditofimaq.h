#ifndef __MWDEMOIMAQ_HEADER__
#define __MWDEMOIMAQ_HEADER__

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>

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
const char *const ETHERNET_DEVICE_STR = "ToF Ethernet Device";

enum { USB_DEVICE_ID = 1, ETHERNET_DEVICE_ID = 2 };

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

enum {
    ADITOF_PROPERTY_MODE = 1,
    ADITOF_PROPERTY_FRAME_TYPE = 2,
    ADITOF_PROPERTY_SMALL_SIGNAL = 3,
    ADITOF_PROPERTY_AFE_TEMP = 4,
    ADITOF_PROPERTY_LASER_TEMP = 5
};

/// Mode enum property
const char *const MODE_STR = "CameraMode";
const char *const MODE_NEAR_STR = "Near";
const char *const MODE_MEDIUM_STR = "Medium";
const char *const MODE_FAR_STR = "Far";

enum { MODE_NEAR_ID = 1, MODE_MEDIUM_ID = 2, MODE_FAR_ID = 3 };

const char *const FRAME_TYPE_STR = "FrameType";
const char *const FRAME_TYPE_DEPTH_STR = "Depth";
const char *const FRAME_TYPE_IR_STR = "Ir";

enum { FRAME_TYPE_DEPTH_ID = 1, FRAME_TYPE_IR_ID = 2 };

const char *const SMALL_SIGNAL_STR = "SmallSignalRemoval";
const int64_t SMALL_SIGNAL_LOWER_LIMIT = 0;
const int64_t SMALL_SIGNAL_UPPER_LIMIT = 16383;
const int64_t SMALL_SIGNAL_DEFAULT = 50;

const char *const AFE_TEMPERATURE_STR = "AfeTemperature";
const char *const LASER_TEMPERATURE_STR = "LaserTemperature";

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
