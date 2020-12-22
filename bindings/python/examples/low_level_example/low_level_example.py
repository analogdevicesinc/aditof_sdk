#
# BSD 3-Clause License
#
# Copyright (c) 2019, Analog Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
import aditofpython as tof
import numpy as np


# Find a camera

system = tof.System()

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
cam1Sensor = camera1.getSensor()

eeproms = []
status = camera1.getEeproms(eeproms)
eeprom1 = eeproms[0]

# Read a couple of bytes from EEPROM
eeprom_read_address = 0x0000;
eeprom_read_data = np.array([0, 0, 0, 0], dtype=np.uint8)
status = eeprom1.read(eeprom_read_address, eeprom_read_data, len(eeprom_read_data))
print("eeprom.read()", status)
print(eeprom_read_data, "at address:", eeprom_read_address)

# Read a couple of bytes from AFE
afe_read_addresses = np.array([0x4001, 0x7c22], dtype=np.uint16)
afe_read_data = np.array([0, 0], dtype=np.uint16)
cam1Sensor.readAfeRegisters(afe_read_addresses, afe_read_data, len(afe_read_data))
print(afe_read_data, "at addresses:", afe_read_addresses)

# Read temperatures from available temperature sensors
cam1TempSensors = []
camera1.getTemperatureSensors(cam1TempSensors)

for sensor in cam1TempSensors:
    name = sensor.getName()
    temperature = []
    status = sensor.read(temperature)
    temperatureToPrint = "Unknown"
    if status == tof.Status.Ok:
        temperatureToPrint = temperature[0]
    print('Sensor: {} reads: {}'.format(name, temperatureToPrint))

