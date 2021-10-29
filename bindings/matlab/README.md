# MATLAB Image Acquisition Toolbox adaptor

### Overview
The SDK provides bindings for MATLAB through the help of a Custom Adaptor which facilitates the use of the SDK with the Image Acquisition Toolbox. More info about custom adaptors for the Image Acquisition Toolbox can be found [here](www.mathworks.com/help/imaq/creating-custom-adaptors.html).

### Building and using the adaptor

When building the adaptor, specify the cmake option `-DWITH_MATLAB=on`. If MATLAB is not found automatically by cmake, the option `-DMatlab_ROOT_DIR=<path>` should be used to specifiy the path to the matlab installation folder.

The output of the build is a dymanic library (.so for Linux and .dll for Windows) that is to be loaded in MATLAB by using the Image Acquisition Toolbox specific functions. The `aditof_imaq.m` script provides a full example on how to use the custom adaptor in MATLAB. Before running the `aditof_imaq.m` script make sure to copy the aditof.so/dll and the aditofadapter.so/dll to the folder where the script is.

The adaptor binaries for Linux and Windows are provided as part of the [SDK releases](https://github.com/analogdevicesinc/aditof_sdk/releases/latest).

### ImaqTool usage

There are two types of network devices available when running the matlab imaq tool, BGR and MONO16.
* BGR is used to obtained collored images, so a collormap is applyed to the depth data for a better visualization. The depth data is stored in 8-bits/3 channels and the ir data is clipped to 8-bits.
* MONO16 is used to obtain raw depth data stored in 16-bit values. It cannot be visualized with imaq tool as it is.

![Display Image](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/hardwareBrowser.PNG)

In the Acquisition parameters menu under 'Device Properties' there are a few controls for processing the data:

![Display Image](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/AquisitionParameters.PNG)

* Camera mode (near,medium,far)
* Depth correction (on/off)
* Frame Type: 
 DepthRgb and IrRgb are used with RGB format in order to visualise the data.
 DepthRaw, IrRaw and DepthIrRaw are used with MONO16 to obtain real depth from the camera. (For DepthIrRaw both data are in the same frame; in the first half is depth and in the second half ir.
 
![Display Image](https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/img/AquisitionParametersFrameTypes.PNG) 

 * Geometry correction (on/off)
 * Ir gamma correction (float value)
 * Noise Reduction threshold (0/16383)

### Troubleshooting

When using the `aditof_imaq.m` file, `imaqregister` might return an error that sounds like this `...aditofadapter.dll is not a valid adaptor`. To fix this issue make sure that `aditof.dll` is in the system environment variable `PATH`.
