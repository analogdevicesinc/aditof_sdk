# Windows User guide

## Setting up the system

### Power on sequence
- Plug the SD card that came in the AD-96TOF1-EBZ box into the DragonBoard410c SD card slot. To benefit from the most recent software updates it is higly recommended to update the SD card with the [latest SD card image](https://github.com/analogdevicesinc/aditof_sdk#supported-embedded-platforms)
- Connect USB cable to the host PC
- Connect the 5V power supply to the camera board and set the camera power switch S2 to on. Once the camera board is powered up the DS1 LED will turn on
- Connect the 12V power supply to the DragonBoard 410c. Once power is connected to the DragonBoard the system will boot the Linux OS from the SD card

![Host connections](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/db410c_usb.JPG)

### Enable the USB connection to the host PC
- After the DragonBoard boots up press the S3 button on the camera board. This will start on the DragonBoard the application that manages the USB host interface
- Once the USB connection is active the DS4 LED will light up on the camera board and the USB camera driver will be loaded on the PC enabling the communication between the host PC and the camera system

![Windows driver](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/windows_db410c_usb.JPG)

### Troubleshooting
 - The PC does not load the USB driver after pressing the S3 button on the camera board
    - Press S3 so that DS4 turns off, reconnect the USB cable to the PC into a different USB port, press S3 again so that DS4 turns on
    - Restart the DragonBoard410c while the USB cable is still connected to the host PC

## Running the evaluation application

You can either build the evaluation application from source following the steps described in the ***Building the SDK*** section below or install it using the [aditof-demo installer](https://ci.appveyor.com/project/analogdevicesinc/aditof-sdk/branch/master/artifacts)

![aditof-demo](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/windows_aditof_demo.jpg)

The evaluation application allows to do live streaming of depth and IR data as well as recording the depth and IR data and playing back from a file. The depth data is displayed as a color map ranging from warm to cold colors as the distance from the camera increases. A point in the middle of the depth image shows the distance in mm to the target.

There are 3 operating modes that determine the range of the system:
 - Near - 20cm to 80cm
 - Medium - 80cm to 300cm
 - Far - 300cm to 600cm

When in a certain operating mode the system will measure distances outside of the mode's range but those will not be accurate. 
 
The evaluation application also displays the temperature in deg C of the camera (AFE) and laser boards as read from the temperature sensors installed on each board.

The framerate at which data is acquired from the system is constantly updated on the GUI. The camera board outputs data at 30 frames per second (fps), but due to USB connection limitations the host PC acquires the frames at a lower rate.

## Building the SDK

### SDK only

#### Pre-requisites
* Install MS Visual Studio 14 2015
* Install MS .NET Framework 4.5
* Glog

#### Installing the dependencies
* Glog:
1. git clone https://github.com/google/glog
2. cd glog
3. git checkout tags/v0.3.5
4. mkdir build_0_3_5 && cd build_0_3_5
5. cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=./local_path/glog -G "Visual Studio 14 2015 Win64" ..
6. cmake --build . --target install --config Debug
7. cmake --build . --target install --config Release

#### Download and build SDK only
* Follow below steps to download and generate MS Visual Studio project
1. git clone https://github.com/analogdevicesinc/aditof_sdk
2. cd aditof_sdk
3. mkdir build
4. cd build
5. cmake -G "Visual Studio 14 2015 Win64" -DWITH_EXAMPLES=off ..
6. cmake --build . --config Release

### SDK with examples

#### Additional pre-requisites
* OpenCV

#### Installing the additional dependencies
* OpenCV:
1. Install the latest release of opencv from: https://opencv.org/releases/
2. Then the following OpenCV environment variables need to be set:

* OPENCV_DIR=path_to_opencv_installation_dir\build
* OPENCV_PATH=path_to_opencv_installation_dir\build\x64\vc14\bin

For instance, if OpenCV were to be install at: C:\opencv, then the variable should look like this:
* OPENCV_DIR=C:\opencv\build
* OPENCV_PATH=C:\opencv\build\x64\vc14\bin

#### Build SDK with examples
1. cd aditof_sdk
2. scripts/generate_msvc2015_solution.bat
3. Open 'adi_tof_project.sln' generated in 'aditof_sdk/build' in MS Visual Studio 2015
4. Select 'Release' build
5. Application binary are created in 'aditof_sdk/build/example/aditof-demo/Release' directory
