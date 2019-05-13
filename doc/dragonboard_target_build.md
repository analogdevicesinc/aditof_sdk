
# Building SDK and APPS on Dragonboard

#### Installing the Dependencies
1. sudo apt update
2. sudo apt upgrade
3. sudo apt install v4l-utils

#### Download and build UVC App 
Follow below steps to download and build the UVC App 
1. git clone https://github.com/adi-sdg/tof_sdk
2. cd tof_sdk/apps/uvc-app/
3. make clean all

* Note: You can also copy and run [this](./scripts/db410c_get_target_sw.sh) script on Dragonboard 410 and it will take care of all of the above.

