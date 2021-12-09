# Linux Build Instructions


## Building the SDK only

### Pre-requisites
* CMake
* Glog v0.3.5
* Libwebsockets v3.2.3
  * OpenSSL
* Protocol Buffers v3.9.0

### Installing the dependencies
* CMake:
```console
sudo apt install cmake
```

* Glog:
```console
git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install
```

* Libwebsockets:
```console
sudo apt-get install libssl-dev
git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_2_3 && cd build_3_2_3
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo cmake --build . --target install
```

* protobuf:
```console
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo cmake --build . --target install
```


### Download and build SDK only
```console
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make
```

## SDK with examples

### Additional pre-requisites
* OpenCV

### Installing the additional dependencies
* OpenCV:
```console
sudo apt install libopencv-contrib-dev
sudo apt install libopencv-dev
```

### Build SDK with examples
```console
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_EXAMPLES=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make
```

## SDK with bindings

- Please check the readme files for each type of binding in the [bindings directory](../../bindings).

### Enabling the point cloud display in aditof-demo

 OpenCV needs to be built manually in order to visualize pointCloud data because the prebuild libraries do not come with the viz module installed.
 The steps required for building and including the required dependencies are presented below:
 
 Requirements:
 * OpenCV 4.5.0

```console
sudo apt-get install libgtk-3-dev libvtk6-dev
git clone --branch 4.5.0 --depth 1 https://github.com/opencv/opencv.git
git clone --branch 4.5.0 --depth 1 https://github.com/opencv/opencv_contrib.git
cp -r ./opencv_contrib/modules/viz ./opencv/modules/
cd opencv
mkdir build_4_5_0 && cd build_4_5_0
cmake -DWITH_VTK=ON -DBUILD_opencv_viz=ON -DBUILD_opencv_world=ON -DCMAKE_PREFIX_PATH="/opt/vtk" -DCMAKE_INSTALL_PREFIX="/opt/opencv" ..
sudo cmake --build . --target install
```

Compile the sdk and add OpenCV to CMAKE_PREFIX_PATH:

```console
cd aditof_sdk
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets;/opt/opencv" -DWITH_EXAMPLES=on ..
make
```
