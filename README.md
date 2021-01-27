# Analog Devices 3D ToF software suite 

## Overview
The **ADI ToF SDK** is a cross platform library for the Analog Devices depth cameras based on the ADDI9036 TOF Signal Processor. It provides support for data processing on the embedded processor platform as well as for USB, Ethernet or Wi-Fi to connect to a host computer. This flexibility enables evaluation across a wide range of use cases and environments.

The SDK provides an API to control the ToF camera, IR stream and depth data. Windows and Linux support are built into the SDK as well as sample code and wrappers for various languages including Python, C/C++ and Matlab.

License : [![License](https://img.shields.io/badge/license-BSD_3-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/blob/master/LICENSE)
Platform details : [![Hardware](https://img.shields.io/badge/hardware-wiki-green.svg)](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz)

## Supported host platforms

For more details on building the SDK on a host platform please check the **User Guide** specified below for either Windows OS or Linux OS.

| Operating System | Documentation | GitHub master status | Downloads |
| --------- | ----------- | ----------- | ----------- |
| Windows | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_windows) <br> [Build Instructions](doc/windows/build_instructions.md) | [![Build status](https://ci.appveyor.com/api/projects/status/46t36hmy77ejrf88/branch/master?svg=true)](https://ci.appveyor.com/project/analogdevicesinc/aditof-sdk/branch/master) | [![aditof-demo installer](https://img.shields.io/badge/release-aditof_demo_installer-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/releases/latest) |
| Linux | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_linux) <br> [Build Instructions](doc/linux/build_instructions.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![sdk release](https://img.shields.io/badge/release-sdk-blue.svg)](https://github.com/analogdevicesinc/aditof_sdk/releases/latest) |

## Supported embedded platforms

Designed using a modular approach, the 3D ToF hardware prototyping platform enables connectivity to the 96Boards development boards suite as well RaspberryPI or any other hardware platforms that have the RaspberryPI camera connector. 

For more details on building the SDK on an embedded platform please check the **User Guide** specified below for either DragonBoard 410c or Raspberry Pi.
### 96-TOF1-EBZ
| Platform | Documentation | GitHub master status | Downloads |
| --------- | ------------- | ----------- | ----------- |
| DragonBoard 410c | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_db410c) <br> [Build Instructions](doc/dragonboard410c/build_instructions.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](http://swdownloads.analog.com/cse/aditof/dragonboard410c-latest-image.tar.xz) |
| Raspberry Pi 3,4 | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_rpi) <br> [Build Instructions](doc/raspberrypi3/build_instructions.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](http://swdownloads.analog.com/cse/aditof/raspberrypi-latest-image.tar.xz) |
| Nvidia Jetson Nano | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_jetson) <br> [Build Instructions](doc/jetson/build_instructions.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](http://swdownloads.analog.com/cse/aditof/jetson_latest_image.tar.xz) |
| Nvidia Jetson Xavier NX | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_xavier_nx) <br> [Build Instructions](doc/xavier-nx/build_instructions.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](http://swdownloads.analog.com/cse/aditof/jetsonnx_latest_image.tar.xz) |
| Nvidia Jetson Xavier AGX | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_xavier_agx) <br> [Build Instructions](doc/xavier-agx/build_instructions.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](http://swdownloads.analog.com/cse/aditof/jetson-latest.tar.xz) |
| Thor96 | [Arrow Electronics Github](https://github.com/ArrowElectronics/aditof_sdk#supported-embedded-platforms) | - | - |

### Chicony ToF
| Platform | Documentation | GitHub master status | Downloads |
| --------- | ----------- | ----------- | ----------- |
| Raspberry Pi 3,4 | [User Guide](doc/raspberrypi3/chicony_user_guide.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](http://swdownloads.analog.com/cse/aditof/raspberrypi-chicony-latest-image.tar.xz) |
| Nvidia Jetson Nano | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_jetson) <br> [Build Instructions](doc/jetson/build_instructions.md) | [![Build Status](https://travis-ci.com/analogdevicesinc/aditof_sdk.svg?branch=master)](https://travis-ci.com/analogdevicesinc/aditof_sdk) | [![SD card image](https://img.shields.io/badge/release-latest_sd_card_image-blue.svg)](http://swdownloads.analog.com/cse/aditof/jetson-latest.tar.xz) |

[How to write the SD card image onto the SD card?](doc/sdcard_burn.md)

## SDK documentation

From an architectural point of view, the SDK consists of two layers. One layer is the high level API that allows clients to easily grab a camera object, configure it and request frames. The other layer is the low level API which exposes the interface through which low level operations can be made to the camera hardware.

For more details about the SDK check the links below:

[Software stack documentation](https://github.com/analogdevicesinc/aditof_sdk/blob/master/sdk/readme.md)

[API Doxygen documentation](https://analogdevicesinc.github.io/aditof_sdk/)

[Building and installing the SDK](https://github.com/analogdevicesinc/aditof_sdk/tree/master/cmake/)

## SDK examples
| Example | Language | Description |
| --------- | ------------- | ----------- |
| aditof-demo | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/aditof-demo"> C++ </a> | An application that displays the infrared and depth images. |
| first-frame | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/first-frame"> C++ </a> <br> <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/first_frame"> Python </a> | An example code that shows the steps required to get to the point where camera frames can be captured. |
| first-frame-network | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/examples/first-frame-network"> C++ </a> | An application that shows how to use the aditof sdk to talk to a remote ToF camera over the network. |
| device_example | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/device_example"> Python</a> | A simple example of how to get access to the low-level API of the camera. |
| dnn | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/dnn"> Python with OpenCV</a> <br> <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/opencv/dnn"> C++ with OpenCV </a> | A simple object detection example. |
| imshow | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/opencv/imshow"> C++ with OpenCV </a> | A basic example that displays data provided by the Aditof SDK. |
| Image Acquisition Toolbox | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/matlab"> MATLAB </a> | Examples of how to use the AD-96TOF1-EBZ camera in MATLAB over USB and Network. |
| Hand Gesture Detection | <a href="https://github.com/mathworks/MATLAB-Demo-ADI-ToF"> MATLAB </a> | Detecting hand signs to play Rock, Paper and Scissors. |
| showPointCloud | <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/python/examples/showPointCloud"> Python with Open3D </a> <br> <a href="https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/open3D/showPointCloud"> C++ with Open3D </a> | A basic example that displays a pointcloud built using the Open3D library and the data provided by the Aditof SDK. |

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

