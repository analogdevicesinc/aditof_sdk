# 3D Time of Flight : OpenCV bindings

### Overview
The SDK provides bindings for OpenCV through the helper methods defined in aditof_opencv.h, which provide a way to convert from the `aditof::Frame` data structure to `cv::Mat`.

#### Directory Structure

| Directory/File | Description |
| --------- | ----------- |
| aditof_opencv.h | Contains the helper methods to convert from aditof::Frame to cv::Mat |
| imshow | Contains a basic example that displays data provided by the Aditof SDK |
| CMakeLists.txt | Rules to build the bindings and the examples |