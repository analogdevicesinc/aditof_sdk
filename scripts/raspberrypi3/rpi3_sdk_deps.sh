#!/bin/bash
set -x #echo on

#install the kernel packages
sudo apt update -y
sudo apt upgrade -y
sudo apt install v4l-utils -y
sudo apt install libopencv-dev -y
sudo apt install cmake -y
sudo apt install libssl1.0-dev -y

#create the work folder
rm -rf /home/pi/workspace
cd /home/pi
mkdir workspace && cd workspace
mkdir github && cd github

#install glog
cd /home/pi/workspace/github
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo make -j4 && sudo make install

#install libwebsockets
cd /home/pi/workspace/github
git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_2_3 && cd build_3_2_3
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo make -j4 && sudo make install

#install protobuf
cd /home/pi/workspace/github
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_CXX_FLAGS="-latomic" -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo make -j4 && sudo make install

#download and build the SDK
cd /home/pi/workspace/github
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DRASPBERRYPI=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make -j4
