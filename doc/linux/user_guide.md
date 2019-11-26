# Linux User guide

## Setting up the system

### Required hardware
- [AD-96TOF1-EBZ development kit](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/ad-96tof1-ebz.html)
- [DragonBoard410c](https://www.96boards.org/product/dragonboard410c/)
- DragonBoard410c 12V power supply (e.g. AK-ND-49)
- Micro-USB to USB Cable

### Power on sequence
- Plug the SD card that came in the AD-96TOF1-EBZ box into the DragonBoard410c SD card slot. To benefit from the most recent software updates it is higly recommended to update the SD card with the [latest SD card image](https://github.com/analogdevicesinc/aditof_sdk#supported-embedded-platforms)
- Make sure that switch S6 on the DragonBoard410C is set to SD BOOT (position 2 ON, all others OFF)
- Connect USB cable to the host PC
- Connect the 5V power supply to the camera board and set the camera power switch S2 to on. Once the camera board is powered up the DS1 LED will turn on
- Connect the 12V power supply to the DragonBoard 410c. Once power is connected to the DragonBoard the system will boot the Linux OS from the SD card
- Wait for the board to finish booting. The booting progress can be monitored by observing the user leds 1, 2, 3 and 4 on the DragonBoard410c which are placed between the two USB type A connectors. During boot the leds (especially led 3 and 1) will blink very rapidly. When led 1 is the only one left bliking (about once a second) the boot has finished.

![Host connections](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/db410c_usb.JPG)

### Enable the USB connection to the host PC
- After the DragonBoard boots up press the S3 button on the camera board. This will start on the DragonBoard the application that manages the USB host interface
- Once the USB connection is active the DS4 LED will light up on the camera board and the USB camera driver will be loaded on the PC enabling the communication between the host PC and the camera system
 - To check if the camera has been recongnized by the host PC, open up a terminal and run command:
 ```
 dmesg
 ```
 The ADI TOF DEPTH SENSOR name should come up in the list displayed by dmesg.

![Linux driver](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/linux_db410c_usb.JPG)

### Troubleshooting
 - The PC does not load the USB driver after pressing the S3 button on the camera board
    - Press S3 so that DS4 turns off, reconnect the USB cable to the PC into a different USB port, press S3 again so that DS4 turns on
    - Restart the DragonBoard410c while the USB cable is still connected to the host PC

## Running the evaluation application

You can build the evaluation application from source following the steps described in the ***Building the SDK*** section below.

![aditof-demo](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/aditof_demo.png)

The evaluation application allows to do live streaming of depth and IR data as well as recording the depth and IR data and playing back from a file. The depth data is displayed as a color map ranging from warm to cold colors as the distance from the camera increases. A point in the middle of the depth image shows the distance in mm to the target.

There are 3 operating modes that determine the range of the system:
 - Near - 25cm to 80cm
 - Medium - 30cm to 4.5m (Rev.B: 80cm to 3m)
 - Far - 300cm to 600cm

When in a certain operating mode the system will measure distances outside of the mode's range but those will not be accurate.

The evaluation application also displays the temperature in deg C of the camera (AFE) and laser boards as read from the temperature sensors installed on each board.

The framerate at which data is acquired from the system is constantly updated on the GUI. The camera board outputs data at 30 frames per second (fps), but due to USB connection limitations the host PC acquires the frames at a lower rate.

### Note
 - Use the custom X button which is under the title bar on the top-right side to close the application. Otherwise the application will hang.

### Troubleshooting
- The demo application hangs after closing the main window
  - Due to some limitations the application always hangs if it is closed using the regular X button from the window top bar (title bar). To avoid this unpleasant hang, we've made available a second X button in the top right corner right above the title bar that can be used to safely close the demo application. We hope this to be a temporary workaround.

## Building the SDK

### SDK only

#### Pre-requisites
* CMake
* Glog v0.3.5
* Libwebsockets v3.1
  * OpenSSL
* Protocol Buffers v3.9.0

#### Installing the dependencies
* CMake:
```console
sudo apt install cmake
```

* Glog:
```console
git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install
```

* Libwebsockets:
```console
sudo apt-get install libssl-dev
git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_1 && cd build_3_1
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo cmake --build . --target install
```

* protobuf:
```console
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo cmake --build . --target install
```


#### Download and build SDK only
```console
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make
```

### SDK with examples

#### Additional pre-requisites
* OpenCV

#### Installing the additional dependencies
* OpenCV:
```console
sudo apt install libopencv-contrib-dev
sudo apt install libopencv-dev
```

#### Build SDK with examples
```console
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_EXAMPLES=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make
```
