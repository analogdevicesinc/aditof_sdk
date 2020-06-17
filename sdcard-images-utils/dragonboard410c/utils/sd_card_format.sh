#!/bin/bash
set -x #echo on

#Get tools
wget http://snapshots.linaro.org/96boards/dragonboard410c/linaro/rescue/latest/dragonboard-410c-bootloader-sd-linux-*.zip -O dragonboard410c_bootloader_sd_linux.zip

unzip -d qcom_bootloaders dragonboard410c_bootloader_sd_linux.zip

git clone --branch=master https://git.linaro.org/landing-teams/working/qualcomm/db-boot-tools.git
cd db-boot-tools
git checkout 31972fb

# Depending on the size of the card, change the last argument
# e.g. 32GM card, physical space 29.7G, or 30412M
# Depending on the node of SD card, change XXX 
# e.g. if card is at /dev/sdd, then XXX is sdd
# Depending on the most recent version of qcom_bootloader, '*-sd-linux-xxx' suffix will need to be chnaged in below command
sudo ./mksdcard -o /dev/XXX -p dragonboard410c/linux/sdcard.txt -i ../qcom_bootloaders/dragonboard-410c-bootloader-sd-linux-130/ -s 30412M


