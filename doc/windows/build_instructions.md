# Windows Build Instructions


## Building the SDK only

### Pre-requisites
* Install MS Visual Studio 14 2015
* Install MS .NET Framework 4.5
* CMake
* Glog v0.3.5
* Libwebsockets v3.1
  * OpenSSL
* Protocol Buffers v3.9.0

### Installing the dependencies
* CMake

Windows installer can be downloaded from: https://cmake.org/download/

* Glog:
```console
git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=./local_path/glog -G "Visual Studio 14 2015 Win64" ..
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

* Libwebsockets:
Libewbesockets needs OpenSSL. One option to get it on windows is from: https://slproweb.com/products/Win32OpenSSL.html. Make sure to get the developer package and not the light weight package.
```console
git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_1 && cd build_3_1
cmake -DOPENSSL_ROOT_DIR="C:\OpenSSL-Win64" -DCMAKE_INSTALL_PREFIX=./local_path/websockets -G "Visual Studio 14 2015 Win64" ..
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

* Protobuf:
```console
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX=./local_path/protobuf -G "Visual Studio 14 2015 Win64" ../cmake
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

### Download and build SDK only
* Follow below steps to download the SDK, generate MS Visual Studio project and build it directly from command line
```console
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 14 2015 Win64" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_EXAMPLES=off ..
cmake --build . --config Release
```

## Building the SDK with examples

### Additional pre-requisites
* OpenCV

### Installing the additional dependencies
* OpenCV:
1. Install the latest release of opencv from: https://opencv.org/releases/
2. Then the following OpenCV environment variables need to be set:

```
OPENCV_DIR=path_to_opencv_installation_dir\build
OPENCV_PATH=path_to_opencv_installation_dir\build\x64\vc14\bin
```

For instance, if OpenCV were to be installed at: C:\opencv, then the variable should look like this:
```
OPENCV_DIR=C:\opencv\build
OPENCV_PATH=C:\opencv\build\x64\vc14\bin
```

### Build SDK with examples and in Visual Studio
- Generate the VisualStudio solution
```console
cd aditof_sdk
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 14 2015 Win64" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_EXAMPLES=on ..
```
- Open 'adi_tof_project.sln' generated in 'aditof_sdk/build' in MS Visual Studio 2015
- Select 'Release' build
- Application binaries are created in 'aditof_sdk/build/example/aditof-demo/Release' directory
