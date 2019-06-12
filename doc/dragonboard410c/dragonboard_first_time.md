
# Setting up the DragonBoard™ 410c for the first time
* Note: Instructions assume that the Bootloader package along with Boot/Kernel and Rootfs images are available
* Note: The ADI TOF Mezzanine is not installed on DragonBoard™ 410c during this stage

#### Steps to setup the HW connections
* Connect host computer to DragonBoard™ 410c
1. Power off DragonBoard™ 410c (unplug from power)
2. Ensure that microSD card slot on DragonBoard™ 410c is empty
3. Set S6 switch on DragonBoard™ 410c to ‘0-0-0-0’,  i.e. all switches should be in “off” position
4. Connect USB to microUSB cable from host computer to DragonBoard™ 410c USB OTG port

* Boot DragonBoard™ 410c into fastboot mode
1. Press and hold the Vol (-) button on the DragonBoard™ 410c, this is the S4 button (DragonBoard™ 410c should still NOT be powered on)
2. While holding the Vol (-) button, power on the DragonBoard™ 410c
3. Once DragonBoard™ 410c is powered up, release the Vol (-) button
4. Wait for about 5-10 seconds
5. From the connected Linux host, run below command to check if DragonBoard™ 410c is in fastboot mode:
```
   $ sudo fastboot devices
   "de82318         fastboot"
```
   If you see a fastboot entry as above, then DragonBoard™ 410c is ready to receive the Bootloader, BOOT/KERNEL and ROOTFS images.

#### Flashing the prebuilt BOOT/KERNEL and ROOTFS images onto the Dragonboard-410c
* Flash Bootloader 
1. Open 'Terminal' on Linux host connected to DragonBoard™ 410c, and change directory to location of unzipped Bootloader
2. Run below command to flash the Bootloader
```
   $ sudo ./flashall
```

* Flash boot and rootfs images
1. Open 'Terminal' on Linux host connected to DragonBoard™ 410c, and change directory to location of boot and rootfs images
2. Run below commands to flash the boot and rootfs images on to the Drragonboard™ 410c
```
   $ sudo fastboot flash boot dragonboard410c-boot-linux-4.9.27-camera.img
   $ sudo fastboot flash rootfs linaro-stretch-alip-qcom-snapdragon-arm64-20170510-233.img
```
3. Reboot DragonBoard™ 410c

