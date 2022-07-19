#!/bin/sh

echo "Installing deps..."

sudo apt install python3-pip
sudo pip3 install --upgrade pip
sudo sudo pip3 install opencv-contrib-python

echo "Building SDK with Python bindings"

cd ~/Workspace/aditof_sdk/build
cmake -DWITH_PYTHON=ON ../

echo "Copying files to desktop"
mkdir ~/Desktop/dnn
cp ~/Workspace/aditof_sdk/bindidngs/python/examples/dnn/MobileNetSSD_deploy.prototxt ~/Desktop/dnn
cp ~/Workspace/aditof_sdk/bindidngs/python/examples/dnn/MobileNetSSD_deploy.caffemodel ~/Desktop/dnn
cp ~/Workspace/aditof_sdk/bindidngs/python/examples/dnn/dnn.py ~/Desktop/dnn
cp ~/Workspace/aditof_sdk/build/bindidngs/python/aditofpython.cpython-36m-aarch64-linux-gnu.so ~/Desktop/dnn
