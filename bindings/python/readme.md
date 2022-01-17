# 3D Time of Flight : Python bindings

### Overview
The SDK provides bindings for Python using [pybind11](https://github.com/pybind/pybind11). 

### Generating the library
To generate the python library the following steps must be done:
* build the sdk depending on your platform: https://github.com/analogdevicesinc/aditof_sdk#supported-host-platforms
* go to the build folder and set the WITH_PYTHON flag to on:
##### LINUX
```
cmake -DWITH_PYTHON=on ..
make -j4
```

##### WINDOWS
```
cmake -DWITH_PYTHON=on ..
cmake --build . --config Release (or Debug depending on the configuration) -j 4
```

* the aditof-python dll/so will be generated in build/bindings/python. You must copy the library next to the script
that you want to use.

### Import the library in a script
To use the bindings for a Python project, just import the library:
```python
import aditofpython as tof
```

#### Directory Structure

| Directory/File | Description |
| --------- | ----------- |
| cmake | Contains the cmake files to find pybind11 |
| pybind11 | Pybind11 header only library |
| CMakeLists.txt | Rules to build the Python bindings |
| examples | Python examples |
