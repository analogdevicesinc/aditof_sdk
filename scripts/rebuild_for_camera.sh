#!/bin/bash

#This script can be used to rebuild the SDK depending on various platforms and cameras.
#Its purpose is to be called from a shortcut placed on desktop.
#It expects the parameters passed as the CMAKE command.

#Possible build flag for the camera: -DUSE_FXTOF1=1 
#Possible build flags for the board: -DDRAGONBOARD=1 -DRASPBERRYPI=1 -DJETSON=1 -DXAVIERNX=1 -DXAVIER=1
#The implicit build, if no parameters are passed is for AD-96TOF1-EBZ build on linux.


cd $(dirname $(dirname $(realpath $0)))
git pull
sudo rm -r build
mkdir build
cd build

if [ "$#" -eq 2 ]; then 
	cmake "$1" "$2" -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" .. 

elif [ "$#" -eq 1 ]; then
	cmake "$1" -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" .. 

else
	cmake -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" .. 	
fi

sudo cmake --build . --target install
