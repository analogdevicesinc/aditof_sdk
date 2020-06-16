#!/bin/bash
set -x #echo on

qemu-aarch64-static /bin/bash

su - linaro

sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install v4l-utils -y
sudo apt-get install libopencv-dev -y
sudo apt-get install cmake -y

rm -rf /home/linaro/workspace

cd /home/linaro
mkdir workspace && cd workspace
mkdir github && cd github

git clone https://github.com/google/glog
cd glog
git checkout tags/v0.3.5
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
cmake --build .
sudo cmake --build . --target install

cd /home/linaro/workspace/github
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DDRAGONBOARD=1 ..
make -j8

cd /home/linaro/workspace/github/aditof_sdk/apps/uvc-app
make -j8

cd /home/linaro/workspace/github/aditof_sdk/apps/daemon
sudo cp tof-programming.service /etc/systemd/system/
mkdir build
cd build
cmake ..
make -j8
sudo systemctl enable tof-programming

