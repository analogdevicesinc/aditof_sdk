# Building Kernel for Raspberry Pi™ 3

Official Raspberry Pi kernel building documentation (for reference):
https://www.raspberrypi.org/documentation/linux/kernel/building.md

## Pre-requisites: Dependencies, toolchain, rootfs binaries, kernel source

#### Dependencies
````
sudo apt-get install git build-essential libncurses5-dev libssl-dev ccache
````

#### Toolchain
```
git clone https://github.com/raspberrypi/tools raspi_tools
```

#### Default rootfs Binaries
The RootFS used for RPI is the latest Raspbian image that can be downloaded from here: [raspbian image](https://downloads.raspberrypi.org/raspbian_latest)  
Download and write this to your sdcard:
```
sudo dd if=<Latest Rasbian Image>.img of=/dev/mmcblk0 bs=4M; sync
```

#### ADI Kernel source
```
git clone https://github.com/analogdevicesinc/linux.git
```

## Building the Kernel

```
cd linux # (repo cloned in previous step)
git checkout rpi-4.19.y
export ARCH=arm
export CROSS_COMPILE=< location of ARM toolchain >arm-linux-gnueabihf-
  # ex. '~/raspi_tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-'
export KERNEL_MODULES_PATH=kernel_modules_location
make bcm2709_defconfig
```
If building for specific hardware (ex. ADDI9036), then 'make menuconfig' to enable specific kernel modules.  
ADDI9036 requires VIDEO_ADDI9036 and EEPROM_AT24
```
make menuconfig
```
```
make -j4 dtbs zImage modules
make -j4 modules_install INSTALL_MOD_PATH=${KERNEL_MODULES_PATH}
```

## Copying Kernel Files
Copy device tree binaries and overlays:
```
cp ./arch/arm/boot/dts/*.dtb <sd card boot partition>
cp ./arch/arm/boot/dts/overlays/*.dtb* <sd card boot partition>/overlays/
cp ./arch/arm/boot/dts/overlays/README <sd card boot partition>/overlays/
```
Then, either overwrite the existing kernel image:
```
cp ./arch/arm/boot/zImage <sd card boot partition>/kernel.img
```
Or copy the ADI kernel, and edit  config.txt to boot into it instead
```
cp ./arch/arm/boot/zImage <sd card boot partition>/kernel-adi.img
echo 'kernel=kernel-adi.img' >> <sd card boot partition>/config.txt
```
Add device tree overlays as necessary. ADDI9036 for example:
```
echo 'dtoverlay=rpi-addi9036,revc' >> <sd card boot partition>/config.txt
```

