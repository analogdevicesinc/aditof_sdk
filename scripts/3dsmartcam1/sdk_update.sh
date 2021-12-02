#!/bin/bash

cd ~/Workspace/aditof_sdk/deps

cd glog
sudo rm -rf build_0_3_5
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install
cd ../..

cd libwebsockets
sudo rm -rf build_3_2_3
mkdir build_3_2_3 && cd build_3_2_3
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo cmake --build . --target install
cd ../..

cd protobuf
sudo rm -rf build_3_9_0
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo cmake --build . --target install
cd ../..

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
sudo apt install -y git-lfs
git clone https://github.com/robotics-ai/tof_process_public
cd tof_process_public
git lfs install
git lfs fetch --all
git lfs pull
./box_measure/ADI-Smart-Camera/install_box-measure_dependencies_bionic.sh
./door_sense/ADI-Smart-Camera/install_door-sense_dependencies_bionic.sh
./slam/ADI-Smart-Camera/install_slam_dependencies_bionic.sh
sudo apt install -y --reinstall ./box_measure/ADI-Smart-Camera/aditof-camera-AD-3DSMARTCAM1-PRZ_0.0.1_arm64_nano.deb
sudo apt install -y --reinstall ./box_measure/ADI-Smart-Camera/box-measure_0.0.5_arm64_nano.deb
sudo apt install -y --reinstall ./door_sense/ADI-Smart-Camera/door-sense_0.0.4_arm64_nano.deb
sudo apt install -y --reinstall ./slam/ADI-Smart-Camera/slam_0.0.1_arm64_nano.deb

cat <<EOT >> ~/Desktop/box-measure.desktop
[Desktop Entry]
Type=Link
Name=Box-Measure
Icon=/opt/robotics-ai/box-measure/product-logo.png
OnlyShowIn=LXDE;
URL=/usr/share/applications/box-measure-fx.desktop
EOT

cat <<EOT >> ~/Desktop/door-sense.desktop
[Desktop Entry]
Type=Link
Name=Door-Sense
Icon=/opt/robotics-ai/door-sense/product-logo.png
OnlyShowIn=LXDE;
URL=/usr/share/applications/door-sense-fx.desktop
EOT

cat <<EOT >> ~/Desktop/slam.desktop
[Desktop Entry]
Type=Link
Name=Slam
Icon=/opt/robotics-ai/slam/product-logo.png
OnlyShowIn=LXDE;
URL=/usr/share/applications/slam-fx.desktop
EOT
