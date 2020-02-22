# MATLAB Image Acquisition Toolbox adaptor

### Overview
The SDK provides bindings for MATLAB through the help of a Custom Adaptor which facilitates the use of the SDK with the Image Acquisition Toolbox. More info about custom adaptors for the Image Acquisition Toolbox can be found [here](www.mathworks.com/help/imaq/creating-custom-adaptors.html).

### Building and using the adaptor

When building the adaptor, specify the cmake option `-DWITH_MATLAB=on`. If MATLAB is not found automatically by cmake, the option `-DMatlab_ROOT_DIR=<path>` should be used to specifiy the path to the matlab installation folder.

The output of the build is a dymanic library (.so for Linux and .dll for Windows) that is to be loaded in MATLAB by using the Image Acquisition Toolbox specific functions. The `aditof_imaq.m` script provides a full example on how to use the custom adaptor in MATLAB. Before running the `aditof_imaq.m` script make sure to copy the aditof.so/dll and the aditofadapter.so/dll to the folder where the script is.

The adaptor binaries for Linux and Windows are provided as part of the [SDK releases](https://github.com/analogdevicesinc/aditof_sdk/releases/latest).

### Troubleshooting

When using the `aditof_imaq.m` file, `imaqregister` might return an error that sound like this `...aditofadapter.dll is not a valid adaptor`. To fix this issue make sure that `aditof.dll` is in the system environment variable `PATH`.
