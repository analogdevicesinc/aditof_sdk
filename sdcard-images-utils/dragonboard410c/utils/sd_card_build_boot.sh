#!/bin/bash
set -x #echo on

export ARCH=arm64
export CROSS_COMPILE=../toolchain/bin/aarch64-linux-gnu-
export KERNEL_VERSION=4.9-camera-lt-qcom
export KERNEL_MODULES_PATH=db410c-modules

make defconfig distro.config
make -j$(nproc) Image.gz dtbs modules KERNELRELEASE=${KERNEL_VERSION}
make -j$(nproc) modules_install INSTALL_MOD_PATH=${KERNEL_MODULES_PATH} KERNELRELEASE=${KERNEL_VERSION}

cat arch/$ARCH/boot/Image.gz arch/$ARCH/boot/dts/qcom/apq8016-sbc.dtb > Image.gz+dtb
echo "not a ramdisk" > ramdisk.img
abootimg --create boot-db410c.img -k Image.gz+dtb -r ramdisk.img \
           -c pagesize=2048 -c kerneladdr=0x80008000 -c ramdiskaddr=0x81000000 \
	   -c cmdline="root=/dev/mmcblk1p9 rw rootwait console=ttyMSM0,115200n8"

