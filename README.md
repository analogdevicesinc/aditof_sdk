# Analog Devices 3D ToF software suite 

## Overview
The **ADI ToF SDK** is a cross platform library for the Analog Devices depth cameras based on the ADDI9036 TOF Signal Processor. It provides support for data processing on the embedded processor platform as well as for USB, Ethernet or Wi-Fi to connect to a host computer. This flexibility enables evaluation across a wide range of use cases and environments.

The SDK provides an API to control the ToF camera, IR stream and depth data. Windows and Linux support are built into the SDK as well as sample code and wrappers for various languages including Python, C/C++ and Matlab.

License : [![License](https://img.shields.io/badge/license-GPLv2-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/blob/master/LICENSE)
Platfrom details : [![Hardware](https://img.shields.io/badge/hardware-wiki-green.svg)](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz)

## Supported host platforms

For more details on building the SDK on a host platform please check the **User Guide** specified below for either Windows OS or Linux OS.

| Operating System | Documentation | GitHub master status | Downloads |
| --------- | ----------- | ----------- | ----------- |
| Windows | [User Guide](doc/windows/user_guide.md) | [![Build status](https://ci.appveyor.com/api/projects/status/46t36hmy77ejrf88/branch/master?svg=true)](https://ci.appveyor.com/project/analogdevicesinc/aditof-sdk/branch/master) | [![aditof-demo installer](https://img.shields.io/badge/release-aditof_demo_installer-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/releases/latest) |
| Linux | [User Guide](doc/linux/user_guide.md) | [![Build Status](https://travis-ci.org/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.org/analogdevicesinc/aditof_sdk) | [![sdk release](https://img.shields.io/badge/release-sdk-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/releases/latest) |

## Supported embedded platforms

Designed using a modular approach, the 3D ToF hardware prototyping platform enables connectivity to the 96Boards development boards suite as well RaspberryPI or any other hardware platforms that have the RaspberryPI camera connector. 

For more details on building the SDK on an embedded platform please check the **User Guide** specified below for either DragonBoard 410c or Raspberry PI 3.

| Platform | Documentation | GitHub master status | Downloads |
| --------- | ----------- | ----------- | ----------- |
| DragonBoard 410c | [User Guide](doc/dragonboard410c/user_guide.md) | [![Build Status](https://travis-ci.org/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.org/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-SD_card_image_v0.2-blue.svg)](http://swdownloads.analog.com/cse/aditof/aditof-v0.2-dragonboard410c-sdimage.tar.xz) |
| Raspberry PI 3 | [User Guide](doc/raspberrypi3/user_guide.md) | [![Build Status](https://travis-ci.org/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.org/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-SD_card_image_v0.1-blue.svg)](http://swdownloads.analog.com/cse/aditof/aditof-v0.1-raspberrypi3-sdimage.tar.xz) |

[How to write the SD card image onto the SD card?](doc/sdcard_burn.md)

## SDK documentation

From an architectural point of view, the SDK consists of two layers. One layer is the high level API that allows clients to easily grab a camera object, configure it and request frames. The other layer is the low level API which exposes the interface through which low level operations can be made to the camera hardware.

For more details about the SDK check the links below:

[Software stack documentation](https://github.com/analogdevicesinc/aditof_sdk/blob/master/sdk/readme.md)

[API Doxygen documentation](https://analogdevicesinc.github.io/aditof_sdk/)

## SDK examples

| Example | Language | Description |
| --------- | ----------- | ----------- |
| [aditof-demo](https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/aditof-demo) | C++ |	An application that uses OpenCV to display infrared and depth images |
| [first-frame](https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/first-frame) | C++ | An example code that shows the steps required to get to the point where camera frames can be captured |

## SDK bindings examples

| Example | Language | Description |
| --------- | ----------- | ----------- |
| [dnn](https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/opencv/dnn) | C++ with OpenCV |	Contains a simple object detection example |
| [imshow](https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/opencv/imshow)	| C++ with OpenCV |	Contains a basic example that displays data provided by the Aditof SDK |
| [first_frame.py](https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python) | Python with OpenCV |	Contains a python example to capture a frame |

## Directory Structure
| Directory | Description |
| --------- | ----------- |
| apps | Applications specific to various targets and hosts |
| bindings | SDK bindings to other languages |
| ci | Useful scripts for continuous integration |
| doc | Documentation |
| examples | Example code for the supported programming languages |
| scripts | Useful development scripts |
| sdk | SDK source code |

