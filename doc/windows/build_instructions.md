# Windows Build Instructions


## Building the SDK only

### Pre-requisites
* Install MS Visual Studio 16 2019 or Visual Studio 17 2022
> NOTE: The sdk can be built with both of these IDEs. If you use the second one, just change '16 2019' to '17 2022' wherever it's necessary.
* Install MS .NET Framework 4.5
* CMake
* Glog v0.3.5
* Libwebsockets v3.2.3
  * OpenSSL
* Protocol Buffers v3.9.0

### Installing the dependencies
> NOTE: To speed up the process please replace the value from the -j argument in the make/cmake command
> with the number of cores in your system

* CMake

Windows installer can be downloaded from: https://cmake.org/download/

* Glog:
```console
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=".\local_path\glog" -G "Visual Studio 16 2019" ..
cmake --build . --target install --config Debug -j 4
cmake --build . --target install --config Release -j 4
```

* Libwebsockets:
Libewbesockets needs OpenSSL. One option to get it on windows is from: https://slproweb.com/products/Win32OpenSSL.html. Make sure to get the developer package and not the light weight package.
```console
git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_2_3 && cd build_3_2_3
cmake -DOPENSSL_ROOT_DIR="C:\OpenSSL-Win64" -DCMAKE_INSTALL_PREFIX=".\local_path\websockets" -G "Visual Studio 16 2019" ..
cmake --build . --target install --config Debug -j 4
cmake --build . --target install --config Release -j 4
```

* Protobuf:
```console
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX=".\local_path\protobuf" -G "Visual Studio 16 2019" ../cmake
cmake --build . --target install --config Debug -j 4
cmake --build . --target install --config Release -j 4
```

### Download and build SDK only
* Follow below steps to download the SDK, generate MS Visual Studio project and build it directly from command line
```console
git clone https://github.com/analogdevicesinc/aditof_sdk
cd aditof_sdk
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof_sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof_sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof_sdk\deps\libwebsockets\build_3_2_3\local_path\websockets" -G "Visual Studio 16 2019" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_EXAMPLES=off ..
cmake --build . --config Release -j 4
```
> NOTE: Make sure to replace the prefix path with the one from your disk.

## Building the SDK with examples

### Additional pre-requisites
* OpenCV

### Installing the additional dependencies
* OpenCV:
1. Install the latest release of opencv from: https://opencv.org/releases/
2. Then the following OpenCV environment variables need to be set:

```
OPENCV_DIR=path_to_opencv_installation_dir\build
OPENCV_PATH=path_to_opencv_installation_dir\build\x64\vc15\bin
```

For instance, if OpenCV were to be installed at: C:\opencv, then the variable should look like this:
```
OPENCV_DIR=C:\opencv\build
OPENCV_PATH=C:\opencv\build\x64\vc15\bin
```

### Build SDK with examples and in Visual Studio
- Generate the VisualStudio solution
```console
cd aditof_sdk
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_2_3\local_path\websockets" -G "Visual Studio 16 2019" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_EXAMPLES=on ..
```
- Open 'adi_tof_project.sln' generated in 'aditof_sdk\build' in MS Visual Studio 2019
- Select 'Release' build
- Application binaries are created in 'aditof_sdk\build\example\aditof-demo\Release' directory


## SDK with bindings

- Please check the readme files for each type of binding in the [bindings directory](../../bindings).


## Building the SDK in Visual Studio using the script

### Pre-requisites
* OpenSSL
* OpenCV

### Steps to build the SDK
- Run the script located in: 'aditof_sdk\scripts\windows' with the suitable configuration parameters. <br>
The following example runs the script with generator **Visual Studio 16 2019** and configuration **Release**. The sdk will be built in folder: **current_script_path\build** and the dependencies will be installed in folder: **current_script_path\deps\installed**.
Use parameter *-h* or *--help* for more information regarding the parameters. 
```
.\setup_project.bat -g "Visual Studio 16 2019" -c Release
```
- Open 'adi_tof_project.sln' generated in **current_script_path\build** in MS Visual Studio.

- Select the configuration to match the configuration used in the script. (e.g.,**Release**) for build.
![Display Image](/doc/img/configuration_VS.PNG)

- The sdk Dynamic-link library (aditof.dll) is created in: **current_script_path\build\sdk\Release**.

- Build the application.
![Display Image](/doc/img/build_VS.PNG)

### Steps to run aditof-demo application

- All the above steps used to build the sdk remain valid.

- Setup aditof-demo as start-up project.
![Display Image](/doc/img/startup_VS.PNG)

- Run the application.
![Display Image](/doc/img/run_VS.PNG) 

- Application executable is created in **current_script_path\build\example\aditof-demo\Release** directory.

### Enabling the point cloud display in aditof-demo

 OpenCV needs to be built manually in order to visualize pointCloud data because the prebuild libraries do not come with the viz module installed.
 The steps required for building and including the required dependencies are presented below:
 
 Requirements:
 * OpenCV 4.5.0
   * VTK 8.2

* VTK 8.2
```console
git clone --branch v8.2.0 --depth 1 https://github.com/Kitware/VTK.git
cd VTK
mkdir build_8_2_0 && cd build_8_2_0
cmake -DCMAKE_INSTALL_PREFIX=./local_path/vtk -G "Visual Studio 16 2019" ..
cmake --build . --target install --config Release -j 4
```

* OpenCV
```console
git clone --branch 4.5.0 --depth 1 https://github.com/opencv/opencv.git
git clone --branch 4.5.0 --depth 1 https://github.com/opencv/opencv_contrib.git
cp -r .\opencv_contrib\modules\viz .\opencv\modules\
cd opencv
mkdir build_4_5_0 && cd build_4_5_0
cmake -DWITH_VTK=ON -DBUILD_opencv_viz=ON -DBUILD_opencv_world=ON -DCMAKE_PREFIX_PATH=./local_path/vtk -DCMAKE_INSTALL_PREFIX=./local_path/opencv -G "Visual Studio 16 2019" ..
cmake --build . --target install --config Release -j 4
```
Add the path from ./local_path/vtk/bin to enviroment variables to load them at runtine.

Compile the sdk and add OpenCV to CMAKE_PREFIX_PATH:

```console
cd aditof_sdk
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_2_3\local_path\websockets;C:\projects\aditof-sdk\deps\opencv\build_4_5_0\local_path\opencv" -G "Visual Studio 16 2019" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_EXAMPLES=on ..
cmake --build . --target install --config Release -j 4
```

## Generate doxygen documentation

Requirements:
* Doxygen
* Graphviz

In order to generate the doxygen documentation you must compile the sdk in the following way:
```console
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets;C:\projects\aditof-sdk\opencv\build" -G "Visual Studio 16 2019" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_DOC=on ..
cmake --build . --target doc -j 4
```
After compilation, the documentation can be found at this path:
```console
build\doc\doxygen_doc\html\index.html
```
