## Overview

Build scripts to be ran on Windows.

By default the sdk will be built in folder: **current_script_path\build** using the **Visual Studio 15 2017 Win64** generator and the **Release** configuration.
The dependencies (glog, protobuf and websockets) will be downloaded and installed in **current_script_path\deps\installed**.
Please use option -h or --help for further reference on how to change the defaults.

#Pre-requisites

* CMake (Windows installer can be downloaded from: https://cmake.org/download/)

* OpenCV

1. Install the latest release of opencv from: https://opencv.org/releases/
2. Then the following OpenCV environment variables need to be set:

```
OPENCV_DIR=path_to_opencv_installation_dir\build
OPENCV_PATH=path_to_opencv_installation_dir\build\x64\vc14\bin
```

* OpenSSL. (One option to get it on windows is from: https://slproweb.com/products/Win32OpenSSL.html. Make sure the get the developer package and not the light wheight package.)
