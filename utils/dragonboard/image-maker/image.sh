#!/bin/bash

set -e

yes_or_exit() {
        message=$1
        while true; do
                read -p "${message} [Y/n]" yn
                case $yn in
                        [Yy]* ) break;;
                        [Nn]* ) exit;;
                        * ) echo "Please answer yes or no.";;
                esac
        done
}

answer_yes=""
display_help=""
arch=arm64
cross_compile="toolchain/bin/aarch64-linux-gnu-"
kernel_version="4.9-camera-lt-qcom"
kernel_modules_path="db410c-modules"
branch="master"
image_name=""

while [[ $# -gt 0 ]]
do
key="$1"
case $key in
        -y|--yes)
        answer_yes="True"
        shift # next argument
        ;;

        -h|--help)
        display_help="True"
        shift # next argument
        ;;

        --arch)
        arch=$2
        shift # past argument
        shift # past value
        ;;

        --cross_compile)
        cross_compile=$2
        shift # past argument
        shift # past value
        ;;

        --kernel_version)
        kernel_version=$2
        shift # past argument
        shift # past value
        ;;

        --kernel_modules_path)
        kernel_modules_path=$2
        shift # past argument
        shift # past value
        ;;

        --branch)
        branch=$2
        shift # past argument
        shift # past value
        ;;

        --image_name)
        image_name=$2
        shift # past argument
        shift # past value
        ;;

        *)    # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

echo "########################### configuration ###############################"
echo "arch = ${arch}"
echo "cross_compile = ${cross_compile}"
echo "kernel_version = ${kernel_version}"
echo "kernel_modules-path = ${kernel_modules_path}"
echo "sdk branch/tag = ${branch}"
echo "image name = ${image_name} (if empty the image name will be of format)"
echo "             dragonboard410c_latest_<sha of HEAD commit>"
echo "#########################################################################"

if [[ -z "${answer_yes}" ]]; then
        yes_or_exit "Do you want to continue?"
fi

basedir=$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )

workingdir=".temp"

pushd ${basedir}

mkdir -p ${workingdir}

pushd ${workingdir}

[ -d "aditof_linux" ] || {
        git clone --branch d3/release/ov5640_4.9.27 --depth 1 https://github.com/analogdevicesinc/aditof_linux.git
}

pushd "aditof_linux"

[ -d "toolchain" ] || {
        mkdir -p toolchain
        wget http://snapshots.linaro.org/components/toolchain/binaries/7.2-2017.11-rc1/aarch64-linux-gnu/gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz
        tar -C toolchain -xvf gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz --strip-components 1
}

export ARCH=${arch}
export CROSS_COMPILE=${cross_compile}
export KERNEL_VERSION=${kernel_version}
export KERNEL_MODULES_PATH=${kernel_modules_path}

make defconfig distro.config
make -j$(nproc) Image.gz dtbs modules KERNELRELEASE=${KERNEL_VERSION}
make -j$(nproc) modules_install INSTALL_MOD_PATH=${KERNEL_MODULES_PATH} KERNELRELEASE=${KERNEL_VERSION}

cat arch/$ARCH/boot/Image.gz arch/$ARCH/boot/dts/qcom/apq8016-sbc.dtb > Image.gz+dtb
echo "not a ramdisk" > ramdisk.img
abootimg --create boot-db410c.img -k Image.gz+dtb -r ramdisk.img \
           -c pagesize=2048 -c kerneladdr=0x80008000 -c ramdiskaddr=0x81000000 \
	   -c cmdline="root=/dev/mmcblk1p9 rw rootwait console=ttyMSM0,115200n8"

# copy resolv.conf to run
set +e
pushd /run
sudo mkdir -p resolvconf
sudo cp /etc/resolv.conf resolvconf/
popd # pushd /run
set -e

#get the linux image
wget https://releases.linaro.org/96boards/dragonboard410c/linaro/debian/17.09/linaro-stretch-alip-qcom-snapdragon-arm64-20171016-283.img.gz
gunzip linaro-stretch-alip-qcom-snapdragon-arm64-20171016-283.img.gz
mv linaro-stretch-alip-qcom-snapdragon-arm64-20171016-283.img linaro-4.9.56.img

# resize the image to fit all the things that need to be installed
simg2img linaro-4.9.56.img linaro-4.9.56.img.raw
sudo e2fsck -f linaro-4.9.56.img.raw
sudo resize2fs linaro-4.9.56.img.raw 13G

#mount the image
sudo mount -o loop linaro-4.9.56.img.raw /mnt

#update the kernel modues
sudo rm -rf /mnt/lib/modules/4.9.56-linaro-lt-qcom
sudo cp -rf ${KERNEL_MODULES_PATH}/lib/modules/4.9-camera-lt-qcom /mnt/lib/modules/


########## build sdk

[ -d "aditof_sdk" ] || {
        git clone --branch "${branch}" --depth 1 https://github.com/analogdevicesinc/aditof_sdk
}

sha=""
if [[ -z "${image_name}" ]]; then
        pushd "aditof_sdk"
        sha=$(git log --pretty=format:'%h' -n 1)
        image_name="dragonboard410c_latest_${sha}.img"
        popd
fi

sudo mkdir -p /mnt/home/linaro/workspace
sudo mkdir -p /mnt/home/linaro/workspace/github
sudo cp -r aditof_sdk/ /mnt/home/linaro/workspace/github

touch /mnt/home/linaro/info.txt
echo "sdk version: ${branch} (${sha})" >> /mnt/home/linaro/info.txt

sudo cp ${basedir}/chroot.sh /mnt/
sudo chmod +x /mnt/chroot.sh

sudo cp /usr/bin/qemu-aarch64-static /mnt/usr/bin
sudo modprobe binfmt_misc
for d in dev sys run proc; do sudo mount -o bind /$d /mnt/$d ; done
#sudo chroot /mnt qemu-aarch64-static /bin/bash
cat << EOF | sudo chroot /mnt
./chroot.sh
exit
EOF
########## end build sdk

set +e
#unmount all the partitions in the image
for d in dev sys run proc; do sudo umount /mnt/$d ; done
sudo umount /mnt
set -e

#create the SD card image
img2simg linaro-4.9.56.img.raw linaro-4.9.56-6G.img


wget http://snapshots.linaro.org/96boards/dragonboard410c/linaro/rescue/latest/dragonboard-410c-bootloader-sd-linux-*.zip -O dragonboard410c_bootloader_sd_linux.zip
unzip -d qcom_bootloaders dragonboard410c_bootloader_sd_linux.zip

[ -d "db-boot-tools" ] || {
        git clone --branch=master https://git.linaro.org/landing-teams/working/qualcomm/db-boot-tools.git
}

pushd db-boot-tools
git checkout 31972fb

sudo ./mksdcard -o ${image_name} -p dragonboard410c/linux/sdcard.txt -i ../qcom_bootloaders/dragonboard-410c-bootloader-sd-linux-142/ -s 14000M

mv ${image_name} ../

popd # pushd db-boot-tools

# write to image boot and rootfs

sectorsize=$(fdisk -l ${image_name} | grep "Sector size" | awk '{print $4}')
startboot=$(fdisk -l ${image_name} | grep "${image_name}7" | awk '{print $2}')
startrootfs=$(fdisk -l ${image_name} | grep "${image_name}9" | awk '{print $2}')

rootfslocation=$(expr ${startrootfs} '*' ${sectorsize})
bootlocation=$(expr ${startboot} '*' ${sectorsize})

fdisk -l ${image_name}
echo "----"
echo ${sectorsize}
echo ${startboot}
echo ${startrootfs}
echo ${rootfslocation}
echo ${bootlocation}

foundloop=$(sudo losetup -f)
sudo losetup -o ${bootlocation} ${foundloop} ${image_name}
sudo dd if=boot-db410c.img of=${foundloop}
sudo losetup -d ${foundloop}

foundloop=$(sudo losetup -f)
sudo losetup -o ${rootfslocation} ${foundloop} ${image_name}
sudo simg2img linaro-4.9.56-6G.img ${foundloop}
sudo losetup -d ${foundloop}

popd # pushd "aditof_linux"

popd # pushd ${workingdir}

mv ${workingdir}/"aditof_linux/${image_name}" .

rm -rf ${workingdir}

popd # pushd ${basedir}

set +e
printf -- '\n';
exit 0;
