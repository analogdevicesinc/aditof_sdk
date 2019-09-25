import aditofpython as tof
import numpy as np

system = tof.System()
status = system.initialize()
print("system.initialize()", status)

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
print(types[0])

status = camera1.setMode(modes[0])
print("camera1.setMode()", status)

frame = tof.Frame()
status = camera1.requestFrame(frame)
print("camera1.requestFrame()", status)

frameDetails = tof.FrameDetails()
status = frame.getDetails(frameDetails)
print("frame.getDetails()", status)
print("frame details:", "width:", frameDetails.width, "height:", frameDetails.height, "type:", frameDetails.type)

image = np.array(frame.getData(tof.FrameDataType.Depth), copy=False)
print(image)
