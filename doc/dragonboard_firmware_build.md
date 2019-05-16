# Building Kernel and Rootfs for DragonBoard™ 410c

#### Pre-requisites : Get Kernel source, default rootfs binaries, toolchain and dependencies
```
sudo apt-get install git build-essential fakeroot libncurses5-dev libssl-dev ccache
wget http://snapshots.linaro.org/components/toolchain/binaries/7.2-2017.11-rc1/aarch64-linux-gnu/gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz

git clone https://github.com/analogdevicesinc/aditof_linux.git
cd aditof_linux
git clone https://source.codeaurora.org/quic/kernel/skales
wget https://builds.96boards.org/releases/dragonboard410c/linaro/debian/17.04/initrd.img-4.9.27-linaro-lt-qcom
wget https://releases.linaro.org/96boards/dragonboard410c/linaro/debian/17.04/linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.gz
```

#### Building the Kernel

```
export ARCH=arm64
export CROSS_COMPILE=< location of ARM64 toolchain >aarch64-linux-gnu-
export KERNEL_VERSION=4.9.27-camera-lt-qcom
mkdir kernel_modules
export KERNEL_MODULES_PATH=kernel_modules
make defconfig distro.config
make -j4 kernel dtbs Image modules KERNELRELEASE=${KERNEL_VERSION}
make -j4 modules_install INSTALL_MOD_PATH=${KERNEL_MODULES_PATH} KERNELRELEASE=${KERNEL_VERSION}
skales/dtbTool -o dt.img -s 2048 arch/arm64/boot/dts/qcom/
skales/mkbootimg --kernel arch/arm64/boot/Image --ramdisk initrd.img-4.9.27-linaro-lt-qcom --dt dt.img --pagesize 2048 --base 0x80000000 --cmdline="root=/dev/disk/by-partlabel/rootfs rw rootwait console=ttyMSM0,115200n8" --output ../dragonboard410c-boot-linux-4.9.27-camera.img
```

#### Building the Rootfs

```
simg2img linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw
e2fsck -f linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw
resize2fs linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw 3G
mkdir -p rootfs_ext4/
sudo mount -o loop linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.img.raw rootfs_ext4/
sudo cp -r kernel_modules/lib/* rootfs_ext4/lib/
sudo umount rootfs_ext4/
img2simg linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img
```

#### Updating the Kernel and Roots on DragonBoard™ 410c using fastboot

```
sudo fastboot flash boot dragonboard410c-boot-linux-4.9.27-camera.img
sudo fastboot flash rootfs linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img
```

Note: You can run [this](./scripts/host_setup/db410c_first_time.sh) script to perform all of the above

