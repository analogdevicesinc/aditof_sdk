#!/bin/bash

#get images from ADI server
wget -q "http://swdownloads.analog.com/cse/aditof/xavier_prepare/Image"
if [ $? -ne 0 ]; then
	echo "Could not get Image file. Please check internet connection"
fi

wget -q "http://swdownloads.analog.com/cse/aditof/xavier_prepare/dtb_enc_sig.img";
if [ $? -ne 0 ]; then
	echo "Could not get DTB file. Please check internet connection"
fi

echo "Cloning Linux modules and writing them locally. This may take a while depending on network connection"
wget "http://swdownloads.analog.com/cse/aditof/xavier_prepare/modules.tar.gz";
if [ $? -ne 0 ]; then
	echo "Could not get modules archive file. Please check internet connection"
fi
sudo tar -xf modules.tar.gz -C /
rm modules.tar.gz

# Backup original kernel Image
sudo cp /boot/Image /boot/Image.backup

if [ ! -e Image ]; then
	echo "Step 1 - FAILED: Cannot find patched kernel Image"
	exit 1
else
	echo "Step 1: Copy patched kernel in boot folder"
	sudo cp Image /boot/Image
fi

# Add backup menu entry in bootloader conf
echo "Step 2: Add backup menu entry to bootloader"
cat <<EOF | sudo tee -a /boot/extlinux/extlinux.conf

LABEL backup
    MENU LABEL backup kernel
    LINUX /boot/Image.backup
    INITRD /boot/initrd
    APPEND \${cbootargs}
EOF

# Write encrypted & signed devicetree partition
if [ ! -e dtb_enc_sig.img ]; then
	echo "Step 3 - FAILED: Cannot find encripted devicetree image"
	echo "Revert kernel Image to avoid conflicts"
	sudo cp /boot/Image.backup /boot/Image
	exit 1
else
	echo "Step 3: Write encrypted & signed devicetree partition"
	sudo dd if=dtb_enc_sig.img of=/dev/disk/by-partlabel/kernel-dtb
fi

#Reboot
echo "Step 4: All Set - Board will reboot now"
reboot
