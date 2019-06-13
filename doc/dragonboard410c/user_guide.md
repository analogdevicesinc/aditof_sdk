# DragonBoard410c User guide

## Prerequisites

To run the system in standalone mode, besides the accesories that are provided in the AD-96TOF1-EBZ box you'll need and additional HDMI cable to connect to a monitor and a USB keyboard and mouse.

## Setting up the system

### Power on sequence
- plug the SD card that came in the AD-96TOF1-EBZ box into the DragonBoard410c SD card slot. To benefit from the most recent software updates it is higly recommended to update the SD card with the [latest SD card image](https://github.com/analogdevicesinc/aditof_sdk#supported-embedded-platforms)
- connect the HDMI cable from the monitor to the DragonBoard410c HDMI connector
- connect a USB mouse and keyboard to the DragonBoard410c. It's possible to use either a mouse & keyboard combo or a separate mouse and keyboard
- connect the 5V power supply to the camera board and set the camera power switch S2 to on. Once the camera board is powered up the DS1 LED will turn on
- connect the 12V power supply to the DragonBoard 410c. Once power is connected to the DragonBoard the system will boot the Linux OS from the SD card.

![DB410C connections](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/db410c_standalone.JPG)

### Power off sequence
- under Linux open a terminal and type ***sudo poweroff***. This will safely power off the DragonBoard410c and ensure that the SD card is properly umounted
- remove the 12V supply from the DragonBoard410c
- set the camera board power switch to off

### Troubleshooting
- Linux does not boot
  - Sometimes the SD card is not read correctly and this prevents the system to boot. Reset the system by removing and reapplying power to the DragonBoard410c
  - The SD card is corrupted and this prevents the system from booting. Reflash the SD card with the SD card image.
- The DragonBoard410c restarts after running the ***sudo poweroff*** command
  - This is a frequent issue that prevents the DragonBoard410c to be safely powered off and can lead to the corruption of the SD card. The simplest workaround to avoid SD card corruption is to remove the SD card while the system is running and then remove power from the DragonBoard410c. The other option is to watch carefully the system power state after runnig the poweroff command and immediately after seing that all the LEDs on the DragonBoard410c have turned off remove the power from the DragonBoard.

## Running the evaluation application

Once Linux boots you'll see on the HDMI monitor the Linux desktop and on the top left corner a shortcut to the evaluation application. Double clicking on the icon will start the evaluation application.  A console window will open to show the application's status and, after a few seconds, the evaluation application GUI will be displayed. 

To be continued.... 

## Building the SDK

### SDK dependencies
To build the SDK and run the included applications and example code the following dependencies must be installed in the system:
 - v4l-utils
 - libopencv-dev
 - cmake
 - glog v0.3.5

This [script](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/scripts/db410c_sdk_deps.sh) will get and install all the SDK dependencies and will also download the SDK source code and build it. By default all the code will be stored in */home/linaro/workspace/github*.
 
The SD card image already contains all the SDK dependencies and there's no need to install them again. To update and build the SDK just follow the steps below.

```console
linaro@linaro-alip:~/workspace/github/aditof_sdk$ git pull
linaro@linaro-alip:~/workspace/github/aditof_sdk$ rm -rf build
linaro@linaro-alip:~/workspace/github/aditof_sdk$ mkdir build
linaro@linaro-alip:~/workspace/github/aditof_sdk$ cd build
linaro@linaro-alip:~/workspace/github/aditof_sdk$ cmake -DDRAGONBOARD=1 ..
linaro@linaro-alip:~/workspace/github/aditof_sdk$ make -j8
```
 

