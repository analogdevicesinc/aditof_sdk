#!/bin/bash
set -x #echo on

mkdir toolchain
wget http://snapshots.linaro.org/components/toolchain/binaries/7.2-2017.11-rc1/aarch64-linux-gnu/gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz
tar -C toolchain -xvf gcc-linaro-7.2.1-2017.11-rc1-x86_64_aarch64-linux-gnu.tar.xz --strip-components 1

sudo apt-get update
sudo apt-get install git build-essential fakeroot libncurses5-dev libssl-dev ccache
sudo apt-get install bc
sudo apt-get install libfdt-dev

