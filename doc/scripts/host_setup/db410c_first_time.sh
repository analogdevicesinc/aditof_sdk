#!/bin/bash
set -x #echo on

sudo ./get_kernel_rootfs_dependencies.sh
sudo ./db410c_create_kernel_image.sh
sudo ./db410c_create_rootfs_image.sh
sudo ./db410c_flash_kernel_rootfs.sh

