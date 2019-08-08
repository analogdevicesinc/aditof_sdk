# Raspberry Pi User guide

## Setting up the system

### Required hardware
- [AD-96TOF1-EBZ development kit](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/ad-96tof1-ebz.html)
- [Raspberry Pi](https://www.raspberrypi.org/products/) The system was tested on Raspberry Pi3 Model B V1.2. It can work on other models that have a CSI camera interface input.
- Raspberry Pi 5V power supply
- To run the system in standalone mode, besides the accesories that are provided in the AD-96TOF1-EBZ box you'll need an additional HDMI cable to connect to a monitor and a USB keyboard and mouse
- 15 core flat cable for connection between RPi and AD-96TOF1-EBZ
- jumper wires for connection of secondary I2C lines

### Power on sequence
- plug the SD card into the Raspberry Pi SD card slot. To benefit from the most recent software updates it is higly recommended to update the SD card with the [latest SD card image](https://github.com/analogdevicesinc/aditof_sdk#supported-embedded-platforms)
- connect the HDMI cable from the monitor to the Raspberry Pi HDMI connector
- connect the I2C1 of the Raspberry Pi to AD-96TOF1-EBZ development kit. Please use jumper wires and the table below.
- connect a USB mouse and keyboard to the Raspberry Pi. It's possible to use either a mouse & keyboard combo or a separate mouse and keyboard
- connect the 5V power supply to the camera board and set the camera power switch S2 to on. Once the camera board is powered up the DS1 LED will turn on
- connect the 5V power supply to the Raspberry Pi. Once power is connected to the Raspberry Pi the system will boot the Linux OS from the SD card.

![RPi connections](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/rpi_standalone.JPG)

| Raspberry Pi GPIO Header (J8) | AD-96TOF1-EBZ pin header (P4) |
| ------------- | ------------- |
|  Pin 3 (SDA)  |     Pin 17    |
|               |     Pin 21    |
|  Pin 5 (SCL)  |     Pin 15    |
|               |     Pin 19    |

Take care that the jumper wire connected to RPi header Pin 3 must be split in two and routed to both Pin 17 and Pin 21 on camera PCB. The same for SCL wire

### Power off sequence
- under Linux open a terminal and type ***sudo poweroff***. This will safely power off the Raspberry Pi and ensure that the SD card is properly umounted
- remove the 5V supply from the Raspberry Pi
- set the camera board power switch to off

### Troubleshooting
- Linux does not boot
  - The SD card is corrupted and this prevents the system from booting. Reflash the SD card with the SD card image.

## Running the evaluation application

Once Linux boots you'll see on the HDMI monitor the Linux desktop and on the top left corner a shortcut to the evaluation application. Double clicking on the icon will start the evaluation application.  A console window will open to show the application's status and, after a few seconds, the evaluation application GUI will be displayed.

![aditof-demo](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/db410c_aditof_demo.jpg)

The evaluation application allows to do live streaming of depth and IR data as well as recording the depth and IR data and playing back from a file. The depth data is displayed as a color map ranging from warm to cold colors as the distance from the camera increases. A point in the middle of the depth image shows the distance in mm to the target.

There are 3 operating modes that determine the range of the system:
 - Near - 20cm to 80cm
 - Medium - 80cm to 300cm
 - Far - 300cm to 600cm

When in a certain operating mode the system will measure distances outside of the mode's range but those will not be accurate.

The evaluation application also displays the temperature in deg C of the camera (AFE) and laser boards as read from the temperature sensors installed on each board.

The framerate at which data is acquired from the system is constantly updated on the GUI. The camera board outputs data at 30 frames per second (fps), but due to system load the evaluation application processes the frames at a lower rate.

### Note
 - Use the custom X button which is under the title bar on the top-right side to close the application. Otherwise the application will hang.

### Troubleshooting
- The demo application hangs after closing the main window
  - Due to some limitations the application always hangs if it is closed using the regular X button from the window top bar (title bar). To avoid this unpleasant hang, we've made available a second X button in the top right corner right above the title bar that can be used to safely close the demo application. We hope this to be a temporary workaround.

## Building the SDK

### SDK dependencies
To build the SDK and run the included applications and example code the following dependencies must be installed in the system:
 - v4l-utils
 - libopencv-dev
 - cmake
 - glog v0.3.5

This [script](https://github.com/analogdevicesinc/aditof_sdk/blob/master/scripts/raspberrypi3/rpi3_sdk_deps.sh) will get and install all the SDK dependencies and will also download the SDK source code and build it. By default all the code will be stored in */home/pi/workspace/github*.

The SD card image already contains all the SDK dependencies and there's no need to install them again. To update and build the SDK just follow the steps below.

```console
pi@raspberry:~/workspace/github/aditof_sdk$ git pull
pi@raspberry:~/workspace/github/aditof_sdk$ rm -rf build
pi@raspberry:~/workspace/github/aditof_sdk$ mkdir build
pi@raspberry:~/workspace/github/aditof_sdk$ cd build
pi@raspberry:~/workspace/github/aditof_sdk$ cmake -DRASPBERRYPI=1 ..
pi@raspberry:~/workspace/github/aditof_sdk$ make -j4
```
***Note:*** if the above commands return permission errors try running them as *sudo*

## Linux Kernel
A customized [kernel](https://github.com/analogdevicesinc/linux/tree/adi-4.19.0) is provided for the Raspberry Pi including the [V4L2 driver for the ADDI9036](https://github.com/analogdevicesinc/linux/blob/adi-4.19.0/drivers/media/i2c/addi9036.c) and other improvements to support all the ADI depth camera features.

