#!/bin/bash
set -x #echo on

#install the kernel packages
sudo apt update -y
sudo apt upgrade -y
sudo apt install v4l-utils -y
sudo apt install libopencv-dev -y
sudo apt install cmake -y

#create the work folder
rm -rf /home/linaro/workspace
cd /home/linaro
mkdir workspace && cd workspace
mkdir github && cd github

#install glog
git clone https://github.com/google/glog
cd glog
git checkout tags/v0.3.5
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install

#download and build the SDK
cd /home/linaro/workspace/github
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DDRAGONBOARD=1 ..
make -j8

#build the uvc-app for host USB connectivity
cd /home/linaro/workspace/github/aditof_sdk/apps/uvc-app
make -j8

#build and install the USB daemon
cd /home/linaro/workspace/github/aditof_sdk/apps/daemon
sudo cp tof-programming.service /etc/systemd/system/
mkdir build
cd build
cmake ..
make -j8
sudo systemctl enable tof-programming
