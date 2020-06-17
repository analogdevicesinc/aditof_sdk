#!/bin/bash
set -x #echo on

cd dragonboard410c_bootloader_emmc_linux-88
sudo ./flashall
cd ..

sleep 1

sudo fastboot reboot

sleep 5

sudo fastboot flash boot boot-linaro-buster-dragonboard-410c-528.img
sudo fastboot flash rootfs dragonboard410c-rootfs-debian-stretch-alip-20170630-13.emmc.img
