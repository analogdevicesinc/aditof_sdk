import aditofpython as tof
import numpy as np


# Find a camera

system = tof.System()
status = system.initialize()
print("system.initialize()", status)

cameras = []
status = system.getCameraList(cameras)
print("system.getCameraList()", status)


# Setup the camera to get it up and running

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


# Get access to the low-level API of the camera

device1 = camera1.getDevice()

# Read a couple of bytes from EEPROM
eeprom_read_address = 0x0000;
eeprom_read_data = np.array([0, 0, 0, 0], dtype=np.uint8)
status = device1.readEeprom(eeprom_read_address, eeprom_read_data, len(eeprom_read_data))
print("device1.readEeprom()", status)
print(eeprom_read_data, "at address:", eeprom_read_address)

# Read a couple of bytes from AFE
afe_read_addresses = np.array([0x4001, 0x7c22], dtype=np.uint16)
afe_read_data = np.array([0, 0], dtype=np.uint16)
device1.readAfeRegisters(afe_read_addresses, afe_read_data, len(afe_read_data))
print(afe_read_data, "at addresses:", afe_read_addresses)

# Read the AFE and laser temperatures
afe_temp = []
status = device1.readAfeTemp(afe_temp)
print("device1.readAfeTemp()", status)
print("AFE temperature:", afe_temp, status)
laser_temp = []
status = device1.readLaserTemp(laser_temp)
print("device1.readLaserTemp()", status)
print("Laser temperature:", afe_temp, status)
