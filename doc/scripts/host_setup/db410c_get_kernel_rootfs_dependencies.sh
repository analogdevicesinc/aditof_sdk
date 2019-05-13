#!/bin/bash
set -x #echo on

sudo apt-get update
sudo apt-get install git build-essential fakeroot libncurses5-dev libssl-dev ccache
sudo apt-get install libfdt-dev

# Create the Base directory for maintaining relative paths
mkdir dragonboard_setup && cd dragonboard_setup

# Get toolchain
mkdir toolchain
wget http://snapshots.linaro.org/components/toolchain/binaries/7.2-2017.11-rc1/aarch64-linux-gnu/gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz
tar -C toolchain -xvf gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz --strip-components 1
rm -frv gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz

# Get Kernel source code, rootfs binaries, and boot image tools
git clone https://github.com/adi-sdg/dragonboard_410c_kernel.git
wget https://builds.96boards.org/releases/dragonboard410c/linaro/debian/17.04/initrd.img-4.9.27-linaro-lt-qcom
wget https://releases.linaro.org/96boards/dragonboard410c/linaro/debian/17.04/linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.gz
git clone https://source.codeaurora.org/quic/kernel/skales
mkdir kernel_modules

cd ../











