#!/bin/bash

#get images from ADI server
wget -q "http://swdownloads.analog.com/cse/aditof/xavier_prepare/Image"
if [ $? -ne 0 ]; then
	echo "Could not get Image file. Please check internet connection"
fi

wget -q "http://swdownloads.analog.com/cse/aditof/xavier_prepare/tegra194-p2888-0001-p2822-0000-addi9036.dtb";
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
	rm Image
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
if [ ! -e tegra194-p2888-0001-p2822-0000-addi9036.dtb ]; then
	echo "Step 3 - FAILED: Cannot find encripted devicetree image"
	echo "Revert kernel Image to avoid conflicts"
	sudo cp /boot/Image.backup /boot/Image
	exit 1
else
	echo "Step 3: Copy DTB to boot folder and set bootloader to load it"
	sudo cp tegra194-p2888-0001-p2822-0000-addi9036.dtb /boot/
	sudo sed -i '0,/LINUX/!b;//a\      FDT /boot/tegra194-p2888-0001-p2822-0000-addi9036.dtb' /boot/extlinux/extlinux.conf
	rm tegra194-p2888-0001-p2822-0000-addi9036.dtb
fi

#Reboot
echo "Step 4: All Set - Board will reboot now"
reboot
