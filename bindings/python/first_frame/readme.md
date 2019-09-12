# 3D Time of Flight : Python bindings

### Overview
This example shows how to capture a frame using the SDK and Python.

NumPy library is needed for scientific computations. 
If NumPy library is not found you should install it using pip (a package manager for Python)
``` pip install numpy
```

For initializing the system
```python
system = tof.System()
status = system.initialize()
```

Getting a camera from the system, configuring the mode and the frame type and initializing it
```python
cameras = []
status = system.getCameraList(cameras)
print("system.getCameraList()", status)

camera1 = cameras[0]

modes = []
status = camera1.getAvailableModes(modes)
print("system.getAvailableModes()", status)
print(modes)

types = []
status = camera1.getAvailableFrameTypes(types)
print("system.getAvailableFrameTypes()", status)
print(types)

status = camera1.initialize()
print("camera1.initialize()", status)

camDetails = tof.CameraDetails()
status = camera1.getDetails(camDetails)
print("system.getDetails()", status)
print("camera1 details:", "id:", camDetails.cameraId, "connection:", camDetails.connection)

status = camera1.setFrameType(types[0])
print("camera1.setFrameType()", status)

status = camera1.setMode(modes[0])
print("camera1.setMode()", status)
```

Getting a frame from the camera
```python
frame = tof.Frame()
status = camera1.requestFrame(frame)
print("camera1.requestFrame()", status)
```

