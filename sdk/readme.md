# 3D Time of Flight SDK 

### Overview
The SDK provides a framework for developing applications using the Analog Devices depth sensing technology. The API is split into a high level API for system configuration and data capture and a low level API exposing more advanced harwdare configuration options. The diagram below shows where the SDK fits in the broader software stack and the additional software components that are required to enable USB and network connectivity to a host PC.

![Software stack](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/sdk_software_stack.png)

#### Embedded platform software components
- V4L2 kernel driver
  - Talks to the ToF mezzanine card through MIPI and SPI/I2C
- Gadget USB video kernel driver
  - Used for the USB interface to the host PC so that the system shows up on the PC as a video device
  - Implements extension units to allow for custom IOCTs
  - Interfaces with the ToF module through the V4L2 driver by the means of the user space UVC application
- Network server
  - Implements the communication protocol with a remote system via Ethernet / WiFi
  - Interfaces with the ToF module through the V4L2 driver
  - Based on websockets & google protocol buffers

#### Host PC software components
- UVC driver
  - Standard UVC driver in the system for both Windows and Linux
- Network client
  - Connects to the network server to access the ToF module
  - Based on websockets & google protocol buffers


### Directory Structure
| Directory/File | Description |
| --------- | ----------- |
| include | Contains the public headers of the SDK |
| src | Contains the source code of the SDK |
| CMakeLists.txt | Rules for building the SDK source code on various platforms |
