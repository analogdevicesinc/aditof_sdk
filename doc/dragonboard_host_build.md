
# Building SDK and APPS on Dragonboard

#### Installing the Dependencies
* Bring the system up-to-date
1. sudo apt update
2. sudo apt upgrade

* Install dependencies
1. sudo apt install v4l-utils
2. sudo apt install libopencv-contrib-dev
3. sudo apt install libopencv-dev
4. sudo apt install cmake

* Build and Install dependencies
1. git clone https://github.com/google/glog
2. cd glog
3. git checkout tags/v0.3.5
4. mkdir build_0_3_5 && cd build_0_3_5
5. cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
6. sudo cmake --build . --target install

#### Download and build SDK and Examples
Follow below steps to download and build the SDK and Examples 
1. git clone https://github.com/adi-sdg/tof_sdk
2. cd tof_sdk
3. mkdir build && cd build
4. cmake .. -DDRAGONBOARD="gnu-machine"
5. make

* Note: You can also copy and run [this](./scripts/db410c_get_host_sw.sh) script on Dragonboard 410 and it will take care of all of the above.
 
