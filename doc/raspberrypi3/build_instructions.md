# Raspberry Pi Build Instructions


## Building the SDK only

### SDK dependencies
To build the SDK and run the included applications and example code the following dependencies must be installed in the system:
 - v4l-utils
 - libopencv-dev
 - cmake
 - glog v0.3.5
 - libwebsockets v3.1
 - protocol buffers v3.9.0

The SD card image already contains all the SDK dependencies and there's no need to install them again. To update and build the SDK just follow the steps below.

```console
pi@raspberry:~/workspace/github/aditof_sdk$ git pull
pi@raspberry:~/workspace/github/aditof_sdk$ cd build
pi@raspberry:~/workspace/github/aditof_sdk/build$ cmake -DRASPBERRYPI=1 ..
pi@raspberry:~/workspace/github/aditof_sdk/build$ make -j4
```

## Linux Kernel
A customized [kernel](https://github.com/analogdevicesinc/linux/tree/adi-4.19.0) is provided for the Raspberry Pi including the [V4L2 driver for the ADDI9036](https://github.com/analogdevicesinc/linux/blob/adi-4.19.0/drivers/media/i2c/addi9036.c) and other improvements to support all the ADI depth camera features.

