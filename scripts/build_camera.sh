#!/bin/bash

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
