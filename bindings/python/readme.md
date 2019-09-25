# 3D Time of Flight : Python bindings

### Overview
The SDK provides bindings for Python using [pybind11](https://github.com/pybind/pybind11). 

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
