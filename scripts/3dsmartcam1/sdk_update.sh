#!/bin/bash

cd ~/Workspace/aditof_sdk/deps

cd glog
sudo rm -rf build_0_3_5
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install
cd ..

cd libwebsockets
sudo rm -rf build_3_1
mkdir build_3_1 && cd build_3_1
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo cmake --build . --target install
cd ..

cd protobuf
sudo rm -rf build_3_9_0
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo cmake --build . --target install
cd ..

cd ~/Workspace/aditof_sdk/
git pull
git fetch
sudo rm -rf build
mkdir build && cd build
cmake -DUSE_3D_SMART=1 -DJETSON=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make -j8
cd ~/Workspace/aditof_sdk/scripts/3dsmartcam1
cp aditof-demo.sh ~/Desktop/
./vnc_install.sh

cd ~/Workspace/
git clone https://github.com/robotics-ai/tof_process_public
cd cd tof_process_public/adi_smart_camera/
./install_bionic_nano.sh
