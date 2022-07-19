#!/bin/sh

echo "Installing deps..."

sudo apt install python3-pip
sudo pip3 install --upgrade pip
sudo sudo pip3 install opencv-contrib-python

echo "Building SDK with Python bindings"

cd ~/Workspace/aditof_sdk/bindings/python/examples/dnn
cmake .

echo "Copying files to desktop"
cp ~/Workspace/aditof_sdk/bindings/python/examples/dnn/MobileNetSSD_deploy.prototxt ~/Desktop
cp ~/Workspace/aditof_sdk/bindings/python/examples/dnn/MobileNetSSD_deploy.caffemodel ~/Desktop
cp ~/Workspace/aditof_sdk/bindings/python/examples/dnn/dnn.py ~/Desktop
