# MacOS Build Instructions


## Building the SDK only

### Pre-requisites
* CMake
* Glog v0.3.5
* Libwebsockets v4.2.0
* Protocol Buffers v3.9.0

### Installing the dependencies
> NOTE: To speed up the process please replace the value from the -j argument in the make/cmake command
> with the number of cores in your system

* CMake:
```console
brew install cmake
```

* Glog:
```console
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo make -j4 && sudo make install
```

* Libwebsockets(without openssl):
```console
git clone --branch v4.2.0 --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_4_2_0 && cd build_4_2_0
cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets  -DLWS_WITH_SSL=OFF ..
sudo make -j4 && sudo make install
```

* protobuf:
```console
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo make -j4 && sudo make install
```


### Download and build SDK only
```console
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make -j4
```

## SDK with examples

### Additional pre-requisites
* OpenCV

### Installing the additional dependencies
* OpenCV:
```console
brew install opencv
brew link opencv
```

### Build SDK with examples
```console
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_EXAMPLES=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make -j4
```

## SDK with bindings

- Please check the readme files for each type of binding in the [bindings directory](../../bindings).

## Generate doxygen documentation

Requirements:
* Doxygen
* Graphviz

```console
brew install doxygen graphviz
```

In order to generate the doxygen documentation you must compile the sdk in the following way:
```console
cmake -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets;/opt/opencv" -DWITH_DOC=on ..
make -j4 doc
```
After compilation, the documentation could be found at this path:
```console
build/doc/doxygen_doc/html/index.html
```
