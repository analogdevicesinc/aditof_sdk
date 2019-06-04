# aditof_sdk [![Build status](https://ci.appveyor.com/api/projects/status/46t36hmy77ejrf88/branch/master?svg=true)](https://ci.appveyor.com/project/analogdevicesinc/aditof-sdk/branch/master)
# Analog Devices 3D ToF software suite

## Overview
The **ADI ToF SDK** is a cross platform library for the Analog Devices depth cameras based on the ADDI9036 TOF Signal Processor. It provides support for data processing on the embedded processor platform as well as for USB, Ethernet or Wi-Fi to connect to a host computer. This flexibility enables evaluation across a wide range of use cases and environments.

The SDK provides an API to control the ToF camera and stream IR and depth data. Windows and Linux support are built into the SDK as well as sample code and wrappers for various languages including Python, C/C++ and Matlab.

## Supported platforms
Designed using a modular approach, the 3D ToF hardware prototyping platform enables connectivity to the 96Boards development boards suite as well RaspberryPI or any other hardware platforms that have the RaspberryPI camera connector. 

| Platform | Getting started guide | SD card image |
| --------- | ----------- | ----------- |
| DragonBoard 410c | [DragonBoard 410c User Guide](doc/dragonboard410c/user_guide.md) | [aditof-v0.2-dragonboard410c-sdimage.tar.xz](http://swdownloads.analog.com/cse/aditof/aditof-v0.2-dragonboard410c-sdimage.tar.xz) |

[How to write the SD card image onto the SD card?](doc/sdcard_burn.md)

## API doxygen documentation
https://analogdevicesinc.github.io/aditof_sdk/

## Directory Structure
| Directory | Description |
| --------- | ----------- |
| apps | Applications specific to various targets and hosts |
| doc | Documentation |
| examples | Example code for the supported programming languages |
| sdk | Software Development Kit source code |
| scripts | Useful development scripts |

