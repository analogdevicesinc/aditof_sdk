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
import sys

system = tof.System()

if len(sys.argv) < 2 :
    print("No ip address provided! Run: python3 first_frame_network.py <ip_address>!")
    exit(1)

ip = sys.argv[1]

cameras = []
status = system.getCameraListAtIp(cameras, ip)
print("system.getCameraListAtIp()", status)

camera1 = cameras[0]
status = camera1.initialize()
print("camera1.initialize()", status)

modes = []
status = camera1.getAvailableModes(modes)
print("system.getAvailableModes()", status)
print(modes)

types = []
status = camera1.getAvailableFrameTypes(types)
print("system.getAvailableFrameTypes()", status)
print(types)

camDetails = tof.CameraDetails()
status = camera1.getDetails(camDetails)
print("camera1.getDetails()", status)
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

#Depth
depthImage = np.array(frame.getData(tof.FrameDataType.Depth), copy=False)
depthImage = depthImage[0: frameDetails.height, 0: frameDetails.width]
print(depthImage)

#Ir
irImage = np.array(frame.getData(tof.FrameDataType.IR), copy=False)
irImage = irImage[0: frameDetails.height, 0: frameDetails.width]
print(irImage)

##Rgb (only for 3d-smart-camera)
#rgbImage = np.array(frame.getData(tof.FrameDataType.RGB), copy=False)
#rgbImage = rgbImage[0: frameDetails.rgbHeight, 0: frameDetails.rgbWidth]
#print(rgbImage)
