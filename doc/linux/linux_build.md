
# Building SDK and APPS on Linux

#### Installing the Dependencies
* Bring the system up-to-date
1. sudo apt update

* Install dependencies
1. sudo apt install libopencv-contrib-dev
2. sudo apt install libopencv-dev
3. sudo apt install cmake

* Build and Install dependencies
1. git clone https://github.com/google/glog
2. cd glog
3. git checkout tags/v0.3.5
4. mkdir build_0_3_5 && cd build_0_3_5
5. cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
6. sudo cmake --build . --target install

#### Download and build SDK and Examples
Follow below steps to download and build the SDK and Examples 
1. git clone https://github.com/analogdevicesinc/aditof_sdk
2. cd aditof_sdk
3. mkdir build && cd build
4. cmake ..
5. make

