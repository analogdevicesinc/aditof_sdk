#!/bin/bash
set -x #echo on

# go to the base directory 
cd dragonboard_setup

# Flash Kernel and Rootfs
sudo fastboot flash boot dragonboard410c-boot-linux-4.9.27-camera.img
sudo fastboot flash rootfs linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img

cd ../


