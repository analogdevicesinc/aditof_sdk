# I.MX8M Mini Build Instructions


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
analog@jetson:~/workspace/aditof_sdk$ git pull
analog@jetson:~/workspace/aditof_sdk$ cd build
analog@jetson:~/workspace/aditof_sdk/build$ cmake -DIMX8MM=1 ..
analog@jetson:~/workspace/aditof_sdk/build$ make -j4
```

## Linux Kernel
A custom kernel image should be compiled for supporting ADI TOF camera on I.MX8MM platform applying the patch
stored in aditof_sdk/misc/imx8mm/imx_4.14.78_1.0.0_ga_var01_one_vc.diff over imx_4.14.78_1.0.0_ga_var01 Variscite kernel release.
