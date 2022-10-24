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
cd /home/linaro/workspace/github
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install

#install libwebsockets
cd /home/linaro/workspace/github
git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_2_3 && cd build_3_2_3
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo cmake --build . --target install

#install protobuf
cd /home/linaro/workspace/github
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo cmake --build . --target install

#download and build the SDK
cd /home/linaro/workspace/github
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DDRAGONBOARD=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
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
