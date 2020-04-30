# showPointCloud example

### Overview

This example demonstrates how to capture data from the TOF system, build a point cloud from it and display it.

#### Creating and showing the point cloud

* Initilize the system and adiToF camera;
* Get the camera intrinsic parameters;
* Enable noise reduction;
* Create and initialize open3D visualizer:
```console
visualization::Visualizer pointcloud_vis;
pointcloud_vis.CreateVisualizerWindow("Pointcloud", 800, 800);
bool is_geometry_added_pointcloud = false;
```
* Initialize *camera::PinholeCameraIntrinsic* with the intrisic parameters:
```console
camera::PinholeCameraIntrinsic intrinsicParameters(frameWidth, frameHeight,
                                                   fx, fy, cx, cy);
```
* Request frame from adiToF camera;
* Transform the depth image into Open3D *geometry::Image* type;
* Transform the IR image into Open3D *geometry::Image* type;
* Transform the 16bits depth image (the depth image raw) into Open3D *geometry::Image* type;
* Color the depth *geometry::Image* with a rainbow spectrum;
* Combine IR *geometry::Image* and the depth *geometry::Image* to get the color *geometry::Image*;
* Create *geometry::RGBDImage* from depth and color images:
```console
auto rgbd_ptr = geometry::RGBDImage::CreateFromColorAndDepth(color_image, depth16bits_image, 1000.0, 3.0, false);
```
* Create the point cloud from the *geometry::RGBDImage* using the intrisic parameters:
```console
pointcloud_ptr = geometry::PointCloud::CreateFromRGBDImage(*rgbd_ptr, intrinsicParameters);
```
* Show in the visualizer the point cloud created:
```console
if (!is_geometry_added_pointcloud) {
    pointcloud_vis.AddGeometry(pointcloud_ptr);
    is_geometry_added_pointcloud = true;
    }
pointcloud_vis.UpdateGeometry();
is_window_closed = pointcloud_vis.PollEvents();
pointcloud_vis.UpdateRender();
```
#### Point cloud example
![Display Image](/doc/img/pointcloud_cpp.png)

