# Windows User guide

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
- Once the USB connection is active the DS4 LED will light up on the camera board and the USB camera driver will be loaded on the PC enabling the communication between the host PC and the camera system. The ADI TOF DEPTH SENSOR may also show up under Cameras instead of Imaging devices.

![Windows driver](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/windows_db410c_usb.JPG)

### Troubleshooting
 - The PC does not load the USB driver after pressing the S3 button on the camera board
    - Press S3 so that DS4 turns off, reconnect the USB cable to the PC into a different USB port, press S3 again so that DS4 turns on
    - Restart the DragonBoard410c while the USB cable is still connected to the host PC

## Running the evaluation application

You can either build the evaluation application from source following the steps described in the ***Building the SDK*** section below or install it using the [aditof-demo installer](https://github.com/analogdevicesinc/aditof_sdk/releases/latest)

Navigate to the location where you chose to install the evaluation application and run aditof-demo.exe. Alternatively, run the shortcut Aditof-Demo from desktop if you enabled the installer to create one.

![aditof-demo](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/aditof_demo.png)

A terminal window will open to display status messages (also warning and error messages, in case there are any issues). Shorty the main window will show up.

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
* Install MS Visual Studio 14 2015
* Install MS .NET Framework 4.5
* CMake
* Glog v0.3.5
* Libwebsockets v3.1
* Protocol Buffers v3.9.0

#### Installing the dependencies
* CMake

Windows installer can be downloaded from: https://cmake.org/download/

* Glog:
```console
git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=./local_path/glog -G "Visual Studio 14 2015 Win64" ..
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

* Libwebsockets:
Libewbesockets needs OpenSSL. One option to get it on windows is from: https://slproweb.com/products/Win32OpenSSL.html. Make sure the get the developer package and not the light wheight package.
```console
git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_1 && cd build_3_1
cmake -DOPENSSL_ROOT_DIR="C:\OpenSSL-Win64" -DCMAKE_INSTALL_PREFIX=./local_path/websockets -G "Visual Studio 14 2015 Win64" ..
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

* Protobuf:
```console
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX=./local_path/protobuf -G "Visual Studio 14 2015 Win64" ../cmake
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

#### Download and build SDK only
* Follow below steps to download and generate MS Visual Studio project
```console
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 14 2015 Win64" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_EXAMPLES=off ..
cmake --build . --config Release
```

### SDK with examples

#### Additional pre-requisites
* OpenCV

#### Installing the additional dependencies
* OpenCV:
1. Install the latest release of opencv from: https://opencv.org/releases/
2. Then the following OpenCV environment variables need to be set:

```
OPENCV_DIR=path_to_opencv_installation_dir\build
OPENCV_PATH=path_to_opencv_installation_dir\build\x64\vc14\bin
```

For instance, if OpenCV were to be install at: C:\opencv, then the variable should look like this:
```
OPENCV_DIR=C:\opencv\build
OPENCV_PATH=C:\opencv\build\x64\vc14\bin
```

#### Build SDK with examples and in Visual Studio
- Generate the VisualStudio solution
```console
cd aditof_sdk
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 14 2015 Win64" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_EXAMPLES=on ..
```
- Open 'adi_tof_project.sln' generated in 'aditof_sdk/build' in MS Visual Studio 2015
- Select 'Release' build
- Application binaries are created in 'aditof_sdk/build/example/aditof-demo/Release' directory
