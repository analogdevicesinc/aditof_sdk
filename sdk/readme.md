# 3D Time of Flight : sdk 

### Overview
The SDK provides a framework for developing applications using the Analog Devices depth sensing technology. The API is spit into a high level API for system configuration and data capture and a low level API exposing more advanced harwdare configuration options. The diagram below shows where the SDK fits in the broader software stack and the additional software components that are required to enable USB and network connectivity to a host PC.

![Software stack](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/sdk_software_stack.png)

### Directory Structure
| Directory/File | Description |
| --------- | ----------- |
| include | Contains the public headers of the SDK |
| src | Contains the source code of the SDK |
| CMakeLists.txt | Rules for building the SDK source code on various platforms |
