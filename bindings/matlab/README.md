# 3D Time of Flight : Matlab bindings

### Overview
The SDK provides bindings for matlab through the help of a Custom Adaptor which facilitates the use of the SDK with the Image Acquisition Toolbox from Matlab.

### Building the bindings

When building the bindings, specify the cmake option `-DWITH_MATLAB=on`. If Matlab is not found automatically by cmake, the option `-DMatlab_ROOT_DIR=<path>` should be used to specifiy the path to the matlab installation folder.
