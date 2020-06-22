# DragonBoard410c Build Instructions


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
linaro@linaro-alip:~/workspace/github/aditof_sdk$ git pull
linaro@linaro-alip:~/workspace/github/aditof_sdk$ cd build
linaro@linaro-alip:~/workspace/github/aditof_sdk/build$ cmake -DDRAGONBOARD=1 ..
linaro@linaro-alip:~/workspace/github/aditof_sdk/build$ make -j4
``` 

## Linux Kernel
A series of [Linaro kernel patches](../../sdcard-images-utils/dragonboard410c/linux-patches) are provided for the DragonBoard410c which include the V4L2 driver for the ADDI903x and other improvements to support all the ADI depth camera features. 
