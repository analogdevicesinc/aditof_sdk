# Scripts

Dragonboard helper scripts to build a `.img` file

#### Prerequisites
The script has been tested on `ubuntu:18.04` and `ubuntu:20.04`, with the following packages being required to be preinstalled before running.
```
sudo apt-get install git wget tar make abootimg simg2img mount qemu-user-static img2simg unzip fdisk build-essential fakeroot libncurses5-dev ccache bc libfdt-dev
```
#### Usage
```
./setup [OPTIONS]

-h|--help
        Print a usage message briefly summarizing the command line options available, then exit.
-y|--yes
        Automatic yes to prompts.
--arch
        Specify the architecture for which the kernel is built.
        Default: arm64
--cross_compile
        Specify the toolchain for the cross compile.
        Default: toolchain/bin/aarch64-linux-gnu-
--kernel_version
        Specify the kernel version.
        Default: 4.9-camera-lt-qcom
--kernel_modules_path
        Specify the kernel modules path.
        Default: db410c-modules
--kernel_repo
        Specify the repo from where the kernel is cloned.
        Default: https://github.com/D3Engineering/linux_kernel_qcomlt.git
--kernel_branch
        Specify the branch that will be used from kernel_repo.
        Default: d3/release/ov5640_4.9.27
--kernel_patches
        Specify the patches that will be applied to kernel_repo.
        Default: kernel_4_9_27
--branch
        Specify the aditof_sdk branch/tag that will be built.
        Default: master
--image_name
        Specify the name of the resulting image.
        Default: dragonboard410c_latest_<sha of HEAD commit from "branch">
```
#### Troubleshooting
1. For the script to work with other `kernel_branch` / `kernel_repo` values make sure to specify the `kernel_patches` which is the folder containing this patches. This folder must be in [the linux patches for dragonboard folder](https://github.com/analogdevicesinc/aditof_sdk/tree/master/sdcard-images-utils/dragonboard410c/linux-patches)
2. In case an error occurs while running the script due to a missing package, or something else, before attempting to run again make sure to remove the folder in which the script temporarily works. The folder should be `.temp` and should be at the same level as the `image.sh` script (../aditof_sdk/sdcard-images-utils/dragonboard410c/utils)
