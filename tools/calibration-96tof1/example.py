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
import numpy as np
import cv2
import pandas as pd
import os
import time
from natsort import natsorted, ns
import logging
import json
import click
import ipaddress

import aditofpython as tof
import tof_calib.gen_delays as gd
import core.frame as frame
import tof_calib.device as device
import cal_eeprom.cal_eeprom as cal_map
import cal_eeprom.firmware_gen as firmware_gen

def get_TAL_values(cam_handle): #Ret: [TAL1_R, TAL2_R, ... , TAL6_F, TAL7_F]
    TAL_val = device.read_AFE_reg(cam_handle, 0xc740, 14)
    for ind, x in enumerate(TAL_val):
        if (x >> 11) == 1:
            #print(ind)
            TAL_val[ind] = TAL_val[ind] | 0xF000
    return TAL_val.astype('int16')

def setup_logging():
    with open('logger.json', 'r') as f:
       config = json.load(f)
       logging.config.dictConfig(config)   


@click.command()
@click.option('--remote', type=click.STRING, help="To connect to a camera over network, specify the ip (e.g. '192.168.1.101')")
def run_example(remote):
    cam_ip = ''
    if remote is not None:
        ip = ipaddress.ip_address(remote)
        print('Running script for a camera connected over Network at ip:', ip)
        cam_ip = remote

    system = tof.System()
    
    #Specify if you want to load firmware from lf files, or from EEPROM
    from_software = True

    if (from_software):
        #Choose firmware path from here
        firmware_path = "config/BM_Kit/Near/"
        
        logger.info("Programming firmware from : " + firmware_path)
        cam_handle = device.open_device2(system, cam_ip)
        device.program_firmware2(cam_handle, firmware_path)
        
        # The following parameters must be specified if firmware is loaded from software
        sw_gain = 1
        sw_offset = 0
    
    else:
        #Specify which mode to load from EEPROM 'near', 'medium', 'far'
        mode = 'near'
        
        cameras = []
        if not cam_ip:
            status = system.getCameraList(cameras)
            logger.info("system.getCameraList()" + str(status))
        else:
            status = system.getCameraListAtIp(cameras, cam_ip)
            logger.info("system.getCameraListAtIp()" + str(status))

        cam_handle = cameras[0]

        modes = []
        status = cam_handle.getAvailableModes(modes)
        logger.info("system.getAvailableModes()" + str(status))
        logger.info(str(modes))

        types = []
        status = cam_handle.getAvailableFrameTypes(types)
        logger.info("system.getAvailableFrameTypes()" + str(status))
        logger.info(str(types))

        status = cam_handle.initialize()
        logger.info("cam_handle.initialize()" + str(status))

        status = cam_handle.setFrameType(types[0])
        logger.info("cam_handle.setFrameType()" + str(status))
        logger.info(types[0])

        status = cam_handle.setMode(mode)
        logger.info("cam_handle.setMode()" + str(status))
        
        frame = tof.Frame()
        
    logger.info("Device Programmed")
    
    i = 0

    # Specify frames to capture to output averaged data
    avg_vals = 1
    values_depth = np.zeros(avg_vals)
    values_ir = np.zeros(avg_vals)
    while(True):
        
        if (from_software):
            # Get depth and ir frames
            depth_image, ir_image = device.get_depth_ir_image(cam_handle)
            # Apply gain and offset
            depth_image = (depth_image*sw_gain)+sw_offset
        else:
            status = cam_handle.requestFrame(frame)
            depth_image = np.array(frame.getData(tof.FrameDataType.Depth),dtype="uint16", copy=False)
            ir_image = np.array(frame.getData(tof.FrameDataType.IR), dtype="uint16", copy=False)

        depth_image_size = (int(depth_image.shape[0] / 2), depth_image.shape[1])
        depth_image_2 = np.resize(depth_image, depth_image_size)
        depth_image_2 = cv2.flip(depth_image_2, 1)
        depth_image_2 = np.uint8(depth_image_2)
        color_depth = cv2.applyColorMap(depth_image_2, cv2.COLORMAP_RAINBOW)
        
        #Draw rectangle at the center of depth image
        cv2.rectangle(color_depth,(315, 235),(325, 245),(0,0,0),2)      

        ir_image_2 = np.uint8(255*(ir_image/4095))
        ir_image_2.resize((480,640))
        ir_image_2 = cv2.flip(ir_image_2, 1)
        ir_image_2 = cv2.applyColorMap(ir_image_2, cv2.COLORMAP_BONE)
        
        #Output images as .png 
        cv2.imwrite('ir.png', ir_image_2)
        cv2.imwrite('depth.png', color_depth)
        
        #Uncomment if running example on 96/Arrow board with HDMI to show stream data on monitor
        #cv2.namedWindow("Depth", cv2.WINDOW_AUTOSIZE)
        #cv2.imshow( "Depth", color_depth)
        #cv2.imshow( "IR", ir_image_2)
        
        #if cv2.waitKey(1) >= 0:
        #        break

        # values_depth[i] = depth_image[240,320]
        # values_ir[i] = ir_image[240,320]
        # i = i + 1
        # # Generates stats for middle pixel
        # if i == avg_vals:
            # Depth = np.average(values_depth)
            # ir = np.average(values_ir)
            # Std = np.std(values_depth)
            # Noise = (Std/Depth) * 100
            # print('Depth:' + str(Depth))
            # print('STD:' + str(Std))
            # print('N%:' + str(Noise))
            # print('IR:' + str(ir))
            # i = 0
            
        print(get_TAL_values(cam_handle))

if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_example()

