# First frame Example

### Overview
This example shows how to use Open3d to create and show a pointcloud from aditof frames 
To install Open3d use pip (a package manager for Python):

```python
pip install open3d
```


NumPy library is needed for scientific computations. 
If NumPy library is not found you should install it using pip: 

```python
pip install numpy
```

#### Creating and showing the point cloud

* Initilize the system and adiToF camera;
* Enable noise reduction;
* Get the camera intrinsic parameters;
* Initialize *o3d.camera.PinholeCameraIntrinsic* with the intrisic parameters:
```console
cameraIntrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)                                                  fx, fy, cx, cy);
```
* Create and initialize open3D visualizer:
```console
vis = o3d.visualization.Visualizer()
vis.create_window("PointCloud", 1600, 1600)
```
* Create the generic point cloud:
```console
point_cloud = o3d.geometry.PointCloud()
```
* Request frame from adiToF camera;
* Color the depth image with a rainbow spectrum;
* Combine IR image and the depth image to get the color image;
* Transform the 16bits depth image (the depth image raw) into Open3D *o3d.geometry.Image* type:
```console
    depth16bits_raw = o3d.geometry.Image(depth16bits_map)
```
* Transform the color image into Open3D *o3d.geometry.Image* type:
```console
	color_raw = o3d.geometry.Image(img_color)
``` 
* Create *o3d.geometry.RGBDImage* from depth and color images:
```console
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth16bits_raw, 1000.0, 3.0, False)
```
* Create the point cloud from the *o3d.geometry.RGBDImage* based on the intrisic parameters:
```console
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cameraIntrinsics)
```
* Show in the visualizer the point cloud created:
```console
point_cloud.points = pcd.points
point_cloud.colors = pcd.colors
if first_time_render_pc:
	vis.add_geometry(point_cloud)
    first_time_render_pc = 0
vis.update_geometry()
vis.poll_events()
vis.update_renderer()
```
#### Point cloud example
![Display Image](/doc/img/pointcloud_python.png)
