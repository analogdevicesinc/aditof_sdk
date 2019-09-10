# 3D Time of Flight : cmake

#### Overview
This folder contains helper cmake in files for the project and also config files for other cmake projects to use in order to find the SDK.

#### Using the CMake build system
In order to build the SDK with cmake, we first need to have all the dependencies installed: glog, protobuf, libwebsockets ( [Check instructions for your system](https://github.com/analogdevicesinc/aditof_sdk/tree/master/doc) ).

After creating a build folder and moving into it `mkdir -p build && cd build`, we can run cmake.

`cmake -D<option> <path_to_aditof_sdk>`

which will generate a Makefile in the build directory. Usefull cmake options are:

| \<option\> | value | default | description |
| --------- | ----------- | ----------- | ----------- |
| WITH_EXAMPLES | on/off | on | Build the examples |
| WITH_PYTHON | on/off | off | Build the python bindings |
| WITH_OPENCV | on/off | off | Build the opencv bindings |
| WITH_DOC | on/off | off | Build the doxygen documentation |
| CMAKE_PREFIX_PATH | \<path\> | Empty | Specifies a path which will be used by the FIND_XXX() commands |
| CMAKE_INSTALL_PREFIX | \<path\> |  /usr/local on UNIX, c:/Program Files on Windows | Installation directory used by `cmake install` |
| PYTHON_EXECUTABLE | \<path\> | Path to default python executable used | Specify which python executable should be used for building the python bindings |

The next command that we need to run is

`make`

which will builder the project. After this command we can also run `make doc` to build the doxygen documentation if the `-DWITH_DOC=on` option was specified.
After this we can run

`sudo make install`

in order to install the SDK in the system.

Example: Consider a user that has the dependencies for the project installed in specific folders in `/opt`: `/opt/glog`, `/opt/protobuf`, `/opt/websockets`, and that wants to install the SDK in `/opt/aditof`, with examples on and all the possible bindings enabled. The following set of commands will do:
```
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_PYTHON=on -DWITH_OPENCV=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_INSTALL_PREFIX="/opt/aditof" ..
make
sudo make install
```

#### Using the SDK with other CMake projects
To use the SDK in your own project, simply add this two lines to your projects CMakeLists.txt and your good to go:
```
find_package(aditof 1.0.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE aditof::aditof)
```

Instead of `aditof::aditof` you could use `${aditof_LIBRARIES}`:
```
find_package(aditof 1.0.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${aditof_LIBRARIES})
``` 

The prerequisite for this to work is to have the SDK installed in the system. If the path for the dependencies of the SDK, or even the SDK is not in the default `PATH` you might need to add a `CMAKE_PREFIX_PATH` that points to the dependencies/sdk when running the cmake command for your project.

Example: Consider a user that has the dependencies for the project installed in specific folders in `/opt`: `/opt/glog`, `/opt/protobuf`, `/opt/websockets` and the SDK in `/opt/aditof` and wants to use the SDK with another project. In the CMakeLists.txt of the project the two lines from above are required to be added and the cmake command should specify

```
-DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets;/opt/aditof"
```