
# Building SDK and APPS on Windows

#### Pre-requisites
* Install MS Visual Studio 14 2015
* Install MS .NET Framework 4.5

#### Installing the Dependencies
* Install dependencies required by the aditof demo app
1. Install the latest release of opencv from: https://opencv.org/releases/

* Build and Install dependencies
1. git clone https://github.com/google/glog
2. cd glog
3. git checkout tags/v0.3.5
4. mkdir build_0_3_5 && cd build_0_3_5
5. cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=./local_path/glog -G "Visual Studio 14 2015 Win64" ..
6. cmake --build . --target install --config Debug
7. cmake --build . --target install --config Releaseq


#### Download and build SDK and Examples
* Follow below steps to download and generate MS Visual Studio project
1. git clone https://github.com/analogdevicesinc/aditof_sdk
2. cd aditof_sdk
3. scripts/generate_msvcX_solution.bat (where X is the version of visual studio installed, ex: 2015)

* Follow below steps to build the SDK and Examples
1. Open 'adi_tof_project.sln' generated in 'aditof_sdk/build' in MS Visual Studio 2015
2. Select 'Release' build
3. Build the solution
4. Application binary are created in 'aditof_sdk/build/example/aditof-demo/Release' directory

