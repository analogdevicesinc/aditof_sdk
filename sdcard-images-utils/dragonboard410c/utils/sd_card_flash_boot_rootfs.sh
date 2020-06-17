#!/bin/bash
set -x #echo on

# Assuming the ramdisk default images are present in parent directory
simg2img linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw
e2fsck -f linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw
resize2fs linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw 5G

mkdir -p rootfs_ext4/
sudo mount -o loop linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw rootfs_ext4/
sudo cp -r db410c-modules/lib rootfs_ext4/
sudo umount rootfs_ext4/
img2simg linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img.raw linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img


# Flash the boot and rootfs images on the SD card, replace XXX with actual SD card node, e.g. sdd
# If SD card is on /dev/sdd, then XXX7 will be sdd7 and XXX9 will be sdd9
sudo dd if=boot-db410c.img of=/dev/XXX7
sudo simg2img linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img /dev/XXX9


