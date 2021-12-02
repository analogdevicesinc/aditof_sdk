# Xavier AGX Build Instructions


## Building the SDK only

### SDK dependencies
To build the SDK and run the included applications and example code the following dependencies must be installed in the system:
 - v4l-utils
 - libopencv-dev
 - cmake
 - glog v0.3.5
 - libwebsockets v3.2.3
 - protocol buffers v3.9.0

The SD card image already contains all the SDK dependencies and there's no need to install them again. To update and build the SDK just follow the steps below.

### AD-96TOF1-EBZ

```console
analog@xavier:~/workspace/aditof_sdk$ git pull
analog@xavier:~/workspace/aditof_sdk$ cd build
analog@xavier:~/workspace/aditof_sdk/build$ cmake -DXAVIER=1 ..
analog@xavier:~/workspace/aditof_sdk/build$ make -j4
```

### AD-FXTOF1-EBZ

```console
analog@xavier:~/workspace/aditof_sdk$ git pull
analog@xavier:~/workspace/aditof_sdk$ cd build
analog@xavier:~/workspace/aditof_sdk/build$ cmake -DXAVIER=1 -DUSE_FXTOF1=1 ..
analog@xavier:~/workspace/aditof_sdk/build$ make -j4
```

## Linux Kernel
The SD card image already contains the customized Linux kernel image and required devicetree to support the ToF Camera connected to CSI connector on Xavier.
If rebuilding the kernel or devicetree is needed please follow the [instructions](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_xavier_agx).
