### Available camera details

Camera details can be obtained by using the getDetails camera function and are presented below:

* cameraID (string): Camera identification.
* mode (string): The mode in which the camera operates.
* frameType (frameDetails): Details about the frames that camera is capturing.
* connection (ConnectionType): The type of connection with the camera.
* intrinsics (struct): 
  *  cameraMatrix (vector(float)): The 3x3 intrinsic parameter matrix (a.k.a K matrix) with values specified in pixel units.
  *  distorsionCoefficients (vector(float))
  *  pixelWidth (float): The width of a sensor unit cell specified in mm.
  *  pixelHeight (float): The height of a sensor unit cell specified in mm.
* depthParameters (struct): 
  *  depthGain (float)
  *  depthOffset (float)
  *  maxDept (int): The maximum distance (in millimeters) the camera can measure in the current operating mode.
  *  minDepth (int): The minimum distance (in millimeters) the camera can measure in the current operating mode.
* bitCount (int): The number of bits used for representing one pixel data.

A detailed version of the above table can be found at: 
https://github.com/analogdevicesinc/aditof_sdk/blob/master/sdk/include/aditof/camera_definitions.h