#!/bin/bash
set -x #echo on

#go to a valid path
cd /usr/local/share/

#get the dependencies
sudo apt update
sudo apt install v4l-utils -y

#get the software
if [ ! -d "aditof_sdk" ]; then
git clone https://github.com/analogdevicesinc/aditof_sdk
fi

#build the software
cd aditof_sdk
git checkout master 
cd apps/uvc-app/
make clean all


