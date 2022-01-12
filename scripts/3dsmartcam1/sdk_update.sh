#!/bin/bash

cd ~/Workspace/aditof_sdk/deps

cd glog
git fetch --all --tags
git checkout tags/v0.3.5
sudo rm -rf build_0_3_5 || true
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo make -j8 && sudo make install
cd ../..

cd libwebsockets
git fetch --all --tags
git checkout tags/v3.2.3
sudo rm -rf build_3_2_3 || true
mkdir build_3_2_3 && cd build_3_2_3
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo make -j8 && sudo make install
cd ../..

cd protobuf
git fetch --all --tags
git checkout tags/v3.9.0
sudo rm -rf build_3_9_0 || true
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo make -j8 && sudo make install
cd ../..

cd ~/Workspace/aditof_sdk/
git checkout v3.1.0
git pull
git fetch --tags
sudo rm -rf build || true
mkdir build && cd build
cmake -DUSE_3D_SMART=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make -j8
cd ~/Workspace/aditof_sdk/scripts/3dsmartcam1
cp aditof-demo.sh ~/Desktop/
./vnc_install.sh
