# 3D Time of Flight : cmake

#### Overview
This folder contains helper cmake.in files for the project and also config files for other cmake projects to use in order to find the SDK.

#### Using the CMake build system
In order to build the SDK with cmake, we first need to have all the dependencies installed: glog, protobuf, libwebsockets ( [Check instructions for your system](https://github.com/analogdevicesinc/aditof_sdk/tree/master/doc) ).

After creating a build folder and moving into it `mkdir -p build && cd build`, we can run cmake.

`cmake -D<option> <path_to_aditof_sdk>`

which will generate all the necessary recipes for building and installing. Useful cmake options are:

| \<option\> | value | default | description |
| --------- | ----------- | ----------- | ----------- |
| WITH_EXAMPLES | on/off | on | Build the examples |
| WITH_DOC | on/off | off | Build the doxygen documentation |
| WITH_PYTHON | on/off | off | Build the python bindings |
| WITH_OPENCV | on/off | off | Build the opencv bindings |
| WITH_MATLAB | on/off | off | Build the matlab bindings |
| WITH_OPEN3D | on/off | off | Build the open3D bindings |
| WITH_ROS | on/off | off | Build the ROS bindings |
| WITH_NETWORK | on/off | on | Build the network interface |
| USE_FXTOF1 | on/off | off | Use FXTOF1 camera |
| USE_3D_SMART | on/off | off | Use 3D Smart camera |
| WITH_TOOLS | on/off | off | Build the tools |
| CMAKE_PREFIX_PATH | \<path\> | Empty | Specifies a path which will be used by the FIND_XXX() commands |
| CMAKE_INSTALL_PREFIX | \<path\> |  /usr/local on UNIX, c:/Program Files on Windows | Installation directory used by `cmake install` |
| PYTHON_EXECUTABLE | \<path\> | Path to default python executable used | Specify which python executable should be used for building the python bindings |
| WITH_REVB | on/off | off | When building for raspberry pi platforms, specify WITH_REVB if you are using hardware revision B, else leave it off |
| IGNORE_TARGET_VERSION | on/off | off | Specifies if the client sdk should ignore the sdk version installed on the target |
| WITH_JS | on/off | off | Build the JavaScript bindings |

#### Building and Installing 

To build the sdk the following command is used:

`cmake --build . [--config <config>] [--target <target>]`

Where `<config>` is the build type: `Debug, Release ...` and target is one of the following:

| \<target\> | description |
| --------- | ----------- |
| install | Install the SDK in the system |
| doc | Build the doxygen documentation |
| copy-dll-bindings | Copy the necessary dll files in the bindings build folder (Only on Windows) |
| copy-dll-example | Copy the necessary dll files in the examples build folder (Only on Windows) |

Example: Consider a user that has the dependencies for the project installed in specific folders in `/opt`: `/opt/glog`, `/opt/protobuf`, `/opt/websockets`, and that wants to install the SDK in `/opt/aditof`, with examples on and all the possible bindings enabled. The following set of commands will do:
```
cd aditof_sdk
mkdir build && cd build
cmake -DWITH_PYTHON=on -DWITH_OPENCV=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_INSTALL_PREFIX="/opt/aditof" ..
sudo cmake --build . --target install
```

The sdk can also be built without the network interface
```
cmake -DWITH_NETWORK=off ..
```
When building with this option, Protobuf and Websockets are no longer dependencies of the sdk

After installing you should run `ldconfig` to update the links/cache that the dynamic loader uses.

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