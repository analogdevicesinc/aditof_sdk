#!/bin/bash
set -x #echo on

# Go to the base directory
cd dragonboard_setup

# Build Kernel and Modules
cd dragonboard_410c_kernel
export ARCH=arm64
export CROSS_COMPILE=../toolchain/bin/aarch64-linux-gnu-
export KERNEL_VERSION=4.9-camera-lt-qcom
export KERNEL_MODULES_PATH=../kernel_modules
make defconfig distro.config
make -j4 kernel dtbs Image modules KERNELRELEASE=${KERNEL_VERSION}
make -j4 modules_install INSTALL_MOD_PATH=${KERNEL_MODULES_PATH} KERNELRELEASE=${KERNEL_VERSION}
cd ../

# Build Kernel Image
skales/dtbTool -o dt.img -s 2048 dragonboard_410c_kernel/arch/arm64/boot/dts/qcom/
skales/mkbootimg --kernel dragonboard_410c_kernel/arch/arm64/boot/Image --ramdisk initrd.img-4.9.27-linaro-lt-qcom --dt dt.img --pagesize 2048 --base 0x80000000 --cmdline="root=/dev/disk/by-partlabel/rootfs rw rootwait console=ttyMSM0,115200n8" --output dragonboard410c-boot-linux-4.9.27-camera.img

cd ../


