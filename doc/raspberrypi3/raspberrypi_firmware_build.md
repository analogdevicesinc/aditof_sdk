# Building Kernel for Raspberry Pi™ 3

#### Pre-requisites : Get Kernel source, default rootfs binaries, toolchain and dependencies
```
sudo apt-get install git build-essential libncurses5-dev libssl-dev ccache

git clone https://github.com/analogdevicesinc/linux.git
cd linux
```

#### Building the Kernel

```
git checkout rpi-4.19.y
export ARCH=arm
export CROSS_COMPILE=< location of ARM toolchain >arm-linux-gnueabihf-
export KERNEL_MODULES_PATH=kernel_modules_location
make bcm2709_defconfig
make -j4 dtbs zImage modules
make -j4 modules_install INSTALL_MOD_PATH=${KERNEL_MODULES_PATH}
cp ./arch/arm/zImage <sd card boot partition>/kernel7.img
```

#### Building the Rootfs

The RootFS used for RPI is actually the latest Raspbian image that can be downloaded from here: [raspbian image](https://downloads.raspberrypi.org/raspbian_latest)

