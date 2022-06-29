# Analog Devices 3D ToF software suite 

## Overview
The **ADI ToF SDK** is a cross platform library for the Analog Devices depth cameras based on the ADDI9036 TOF Signal Processor. It provides support for data processing on the embedded processor platform as well as for USB, Ethernet or Wi-Fi to connect to a host computer. This flexibility enables evaluation across a wide range of use cases and environments.

The SDK provides an API to control the ToF camera and stream data. Windows and Linux support are built into the SDK as well as sample code and wrappers for various languages including Python, C/C++ and Matlab.

License : [![License](https://img.shields.io/badge/license-BSD_3-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/blob/master/LICENSE)

## Supported host platforms

For more details on building the SDK on a host platform please check the **User Guide** specified below for either Windows OS or Linux OS.

| Operating System | Documentation | GitHub master status | Downloads |
| --------- | ----------- | ----------- | ----------- |
| Windows | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_windows) <br> [Build Instructions](doc/windows/build_instructions.md) | [![Build status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![aditof-demo installer](https://img.shields.io/badge/release-aditof_demo_installer-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/releases/latest) |
| Linux | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_linux) <br> [Build Instructions](doc/linux/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![sdk release](https://img.shields.io/badge/release-sdk-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/releases/latest) |

## Supported embedded platforms

Designed using a modular approach, the 3D ToF platforms enable connectivity to the to a variety of embedded processing platforms. For more details on running the SDK on your processing platform of choice please check the corresponding **User Guide** below.

### AD-3DSMARTCAM1-PRZ
Platform details : [![Hardware](https://img.shields.io/badge/hardware-wiki-green.svg)](https://wiki.analog.com/resources/eval/user-guides/ad-3dsmartcam1-prz)

| Documentation | GitHub master status | Downloads |
| ------------- | ----------- | ----------- |
| [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-3dsmartcam1-prz?rev=1637140560#getting_your_system_up_and_running) <br> [Build Instructions](doc/3dsmartcam1/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) |  [![sdk release](https://img.shields.io/badge/release-sdk-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/releases/latest) 

### AD-FXTOF1-EBZ
Platform details : [![Hardware](https://img.shields.io/badge/hardware-wiki-green.svg)](https://wiki.analog.com/resources/eval/user-guides/ad-fxtof1-ebz)

| Platform | Documentation | GitHub master status | Downloads |
| --------- | ------------- | ----------- | ----------- |
| Raspberry Pi 3,4  32bit| [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-fxtof1-ebz/ug_rpi) <br> [Build Instructions](doc/raspberrypi3/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/raspberrypi-fxtof1-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/raspberrypi-fxtof1-latest-image-Sha256sum.txt) 
| Raspberry Pi 4 64bit | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-fxtof1-ebz/ug_rpi) <br> [Build Instructions](doc/raspberrypi3/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/raspberrypi-fxtof1-64bit-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/raspberrypi-fxtof1-64bit-latest-image-Sha256sum.txt) 
| Nvidia Jetson Nano | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-fxtof1-ebz/ug_jetson) <br> [Build Instructions](doc/jetson/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/jetson_nano-fxtof1-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/jetson_nano-fxtof1-latest-image-Sha256sum.txt) 
| Nvidia Jetson Xavier NX | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-fxtof1-ebz/ug_xavier_nx) <br> [Build Instructions](doc/xavier-nx/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/xavier_nx-fxtof1-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/xavier_nx-fxtof1-latest-image-sha256.txt)

### AD-96TOF1-EBZ
Platform details : [![Hardware](https://img.shields.io/badge/hardware-wiki-green.svg)](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz)

| Platform | Documentation | GitHub master status | Downloads |
| --------- | ------------- | ----------- | ----------- |
| DragonBoard 410c | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_db410c) <br> [Build Instructions](doc/dragonboard410c/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/dragonboard410c-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/dragonboard410c-latest-image-Sha256sum.txt) |
| Raspberry Pi 3,4 | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_rpi) <br> [Build Instructions](doc/raspberrypi3/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/raspberrypi-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/raspberrypi-latest-image-Sha256sum.txt) 
| Nvidia Jetson Nano | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_jetson) <br> [Build Instructions](doc/jetson/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/jetson_nano-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/jetson_nano-latest-image-Sha256sum.txt) 
| Nvidia Jetson Xavier NX | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_xavier_nx) <br> [Build Instructions](doc/xavier-nx/build_instructions.md) | [![Build Status](https://dev.azure.com/AnalogDevices/3DToF/_apis/build/status/analogdevicesinc.aditof_sdk?branchName=master)](https://dev.azure.com/AnalogDevices/3DToF/_build?view=runs&branchFilter=262) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](https://swdownloads.analog.com/cse/aditof/jetson_xavier_nx-latest-image.tar.xz) <br> [![Sha256sum Checksum](https://img.shields.io/badge/sha256sum-yellow.svg)](https://swdownloads.analog.com/cse/aditof/jetson_xavier_nx-latest-image-Sha256sum.txt) 
| Thor96 | [Arrow Electronics Github](https://github.com/ArrowElectronics/aditof_sdk#supported-embedded-platforms) | - | - |

[How to write the SD card image onto the SD card?](doc/sdcard_burn.md)

## SDK documentation

From an architectural point of view, the SDK consists of two layers. One layer is the high level API which allows to easily create a camera object, configure it and request frames. The other layer is the low level API which exposes an interface through which advanced configuration operations can be executed on the camera hardware.

For more details about the SDK check the links below:

[Software stack documentation](https://github.com/analogdevicesinc/aditof_sdk/blob/master/sdk/readme.md)

[Camera details documentation](https://github.com/analogdevicesinc/aditof_sdk/blob/master/sdk/src/cameras/readme.md)

[API Doxygen documentation](https://analogdevicesinc.github.io/aditof_sdk/)

[Building and installing the SDK](https://github.com/analogdevicesinc/aditof_sdk/tree/master/cmake/)

## SDK examples
| Example | Language | Description |
| --------- | ------------- | ----------- |
| aditof-demo | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/aditof-demo"> C++ </a> | An application that displays the infrared and depth images. |
| first-frame | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/first-frame"> C++ </a> <br> <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/first_frame"> Python </a> | An example code that shows the steps required to get to the point where camera frames can be captured. |
| first-frame-network | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/first-frame-network"> C++ </a> | An application that shows how to use the aditof sdk to talk to a remote ToF camera over the network. |
| low_level_example | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/low_level_example"> Python</a> | A simple example of how to get access to the low-level API of the camera. |
| dnn | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/dnn"> Python with OpenCV</a> <br> <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/opencv/dnn"> C++ with OpenCV </a> | A simple object detection example. |
| imshow | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/opencv/imshow"> C++ with OpenCV </a> | A basic example that displays data provided by the Aditof SDK. |
| Image Acquisition Toolbox | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/matlab"> MATLAB </a> | Examples of how to use the AD-96TOF1-EBZ camera in MATLAB over USB and Network. |
| Hand Gesture Detection | <a href="https://github.com/mathworks/MATLAB-Demo-ADI-ToF"> MATLAB </a> | Detecting hand signs to play Rock, Paper and Scissors. |
| showPointCloud | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/showPointCloud"> Python with Open3D </a> <br> <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/open3D/showPointCloud"> C++ with Open3D </a> | A basic example that displays a pointcloud built using the Open3D library and the data provided by the Aditof SDK. |
| showPointCloud <br> camera node | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/ros"> C++ with ROS </a> | A simple example of how to use AD-96TOF1-EBZ with the ROS distribution. |

## Directory Structure
| Directory | Description |
| --------- | ----------- |
| apps | Applications specific to various targets and hosts |
| bindings | SDK bindings to other languages |
| ci | Useful scripts for continuous integration |
| doc | Documentation |
| examples | Example code for the supported programming languages |
| misc | Calibration and kernel files |
| scripts | Useful development scripts |
| sdk | SDK source code |
| tools | Camera calibration tools |
| utils | Various utils for the embedded platforms |

## Contributing to the SDK

### Formating

The SDK is formated using <a href="https://packages.ubuntu.com/search?keywords=clang-format-6.0">clang-format 6.0</a>.

Before creating a PR please run `./scrips/format.sh` from the root of the project.

In order to prevent a file from being formated add it to `.clangformatignore`
