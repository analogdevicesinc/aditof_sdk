# Linux Build Instructions


## Building the SDK only

### Pre-requisites
* CMake
* Glog v0.3.5
* Libwebsockets v3.1
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
git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_1 && cd build_3_1
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
