#!/bin/bash
set -x #echo on

CDIR=$(pwd)

cd /run
sudo mkdir resolvconf
sudo cp /etc/resolv.conf resolvconf/
cd $CDIR

#get the linux image
wget https://releases.linaro.org/96boards/dragonboard410c/linaro/debian/17.09/linaro-stretch-alip-qcom-snapdragon-arm64-20171016-283.img.gz
gunzip linaro-stretch-alip-qcom-snapdragon-arm64-20171016-283.img.gz
mv linaro-stretch-alip-qcom-snapdragon-arm64-20171016-283.img linaro-4.9.56.img

#resize the image to fit all the things that need to be installed
simg2img linaro-4.9.56.img linaro-4.9.56.img.raw
sudo e2fsck -f linaro-4.9.56.img.raw
sudo resize2fs linaro-4.9.56.img.raw 6G

#mount the image
sudo mount -o loop linaro-4.9.56.img.raw /mnt

#update the kernel modues
sudo rm -rf /mnt/lib/modules/4.9.56-linaro-lt-qcom
sudo cp -rf ../github/dragonboard_410c_kernel/db410c-modules/lib/modules/4.9-camera-lt-qcom /mnt/lib/modules/

#copy some useful scripts
sudo cp chroot.sh /mnt/
sudo chmod +x /mnt/chroot.sh
sudo cp aditof_demo.sh /mnt/home/linaro/
sudo chmod +x /mnt/home/linaro/aditof_demo.sh

#run chroot
sudo cp /usr/bin/qemu-aarch64-static /mnt/usr/bin
sudo modprobe binfmt_misc
for d in dev sys run proc; do sudo mount -o bind /$d /mnt/$d ; done
#sudo chroot /mnt qemu-aarch64-static /bin/bash
cat << EOF | sudo chroot /mnt
./chroot.sh
exit
EOF

#remove the chroot.sh script from the image
sudo rm /mnt/chroot.sh

#unmount all the partitions in the image
for d in dev sys run proc; do sudo umount /mnt/$d ; done
sudo umount /mnt

#create the SD card image
img2simg linaro-4.9.56.img.raw linaro-4.9.56-6G.img
