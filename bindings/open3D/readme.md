# 3D Time of Flight : Open3D bindings

### Overview
The SDK provides bindings for Open3D through the helper methods defined in aditof_open3d.h, which provide a way to convert from the `aditof::Frame` data structure to `open3d::geometry::Image`.

###Instalation
To install open3D go to: [Open3D](http://www.open3d.org/docs/release/tutorial/C++/cplusplus_interface.html)

#### Directory Structure

| Directory/File | Description |
| --------- | ----------- |
| aditof_opencv.h | Contains the helper methods to convert from aditof::Frame to open3d::geometry::Image |
| showPointCloud | Contains a basic example that displays a pointcloud build with data from aditof|
| CMakeLists.txt | Rules to build the bindings and the examples |
