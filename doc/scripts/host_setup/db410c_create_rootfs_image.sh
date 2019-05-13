#!/bin/bash
set -x #echo on

# Go to the base directory
cd dragonboard_setup

simg2img linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw
e2fsck -f linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw
resize2fs linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw 3G
mkdir -p rootfs_ext4/
sudo mount -o loop linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw rootfs_ext4/
sudo cp -r kernel_modules/lib/* rootfs_ext4/lib/
sudo umount rootfs_ext4/
img2simg linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img

cd ..

