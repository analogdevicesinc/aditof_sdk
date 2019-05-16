#!/bin/bash
set -x #echo on

#go to a valid path
cd /usr/local/share/

#get the dependencies
sudo apt update -y
sudo apt install v4l-utils -y
sudo apt install libopencv-contrib-dev -y
sudo apt install libopencv-dev -y
sudo apt install cmake -y

#get the software
if [ ! -d "aditof_sdk" ]; then
git clone https://github.com/analogdevicesinc/aditof_sdk
fi

#build and install dependencies
git clone https://github.com/google/glog
cd glog
git checkout tags/v0.3.5
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install
cd ../../

#build the software
cd aditof_sdk
git checkout master 
mkdir build && cd build
cmake .. -DDRAGONBOARD="410c"
make
