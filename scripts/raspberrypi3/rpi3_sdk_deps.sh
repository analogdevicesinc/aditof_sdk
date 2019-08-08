#!/bin/bash
set -x #echo on

#install the kernel packages
sudo apt update -y
sudo apt upgrade -y
sudo apt install v4l-utils -y
sudo apt install libopencv-dev -y
sudo apt install cmake -y

#create the work folder
rm -rf /home/pi/workspace
cd /home/pi
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
cd /home/pi/workspace/github
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DRASPBERRYPI=1 ..
make -j4
