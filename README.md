# aditof_sdk [![Build status](https://ci.appveyor.com/api/projects/status/46t36hmy77ejrf88/branch/master?svg=true)](https://ci.appveyor.com/project/analogdevicesinc/aditof-sdk/branch/master)
Analog Devices 3D ToF software suite

#### Overview
The **ADI ToF SDK** is a cross platform library for the Analog Devices depth cameras based on the ADDI9036 TOF Signal Processor. It provides support for data processing on the embedded processor platform as well as for USB, Ethernet or Wi-Fi to connect to a host computer, this flexibility enables evaluation across a wide range of use cases and environments.

The SDK provides an API to control the ToF camera and stream IR and depth data. Windows and Linux support are built into the SDK as well as sample code and wrappers for various languages including Python, C/C++ and Matlab.

#### Supported platforms
Designed using a modular approach, the 3D ToF hardware prototyping platform enables connectivity to the 96Boards development boards suite as well RaspberryPI or any other hardware platforms that have the RaspberryPI camera connector. 

| Platform | SD card image |
| --------- | ----------- |
| DragonBoard 410c | [aditof-v0.2-dragonboard410c-sdimage.tar.xz](http://swdownloads.analog.com/cse/aditof/aditof-v0.2-dragonboard410c-sdimage.tar.xz) |

#### API doxygen documentation
https://analogdevicesinc.github.io/aditof_sdk/

#### Directory Structure
| Directory | Description |
| --------- | ----------- |
| apps | Contains the source code of application to be built and executed on target |
| doc | Contains the documentation |
| examples | Contains the source code of examples to be built and executed on Hosts |
| sdk | Contains the source code of software development kit used by examples |
| scripts | Contains scripts that are used when developing |

