import numpy as np
import cv2
import pandas as pd
import os
import time
from natsort import natsorted, ns
import logging
import json

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

if __name__ == "__main__":

    setup_logging()
    logger = logging.getLogger(__name__)
       
    system = tof.System()
    status = system.initialize()
    
    #Specify if you want to load firmware from lf files, or from EEPROM
    from_software = True
    
    if (from_software):
        #Choose firmware path from here
        firmware_path = "config/BM_Kit/Near/"
        
        logger.info("Programming firmware from : " + firmware_path)
        cam_handle = device.open_device2(system)
        device.program_firmware2(cam_handle, firmware_path)
        
        # The following parameters must be specified if firmware is loaded from software
        sw_gain = 1
        sw_offset = 0
    
    else:
        #Specify which mode to load from EEPROM 'near', 'medium', 'far'
        mode = 'near'
        
        cameras = []
        status = system.getCameraList(cameras)
        logger.info("system.getCameraList()" + str(status))

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
            depth_image = np.array(frame.getData(tof.FrameDataType.Depth), copy=True)
            ir_image = np.array(frame.getData(tof.FrameDataType.IR), copy=True)
            
        max = np.amax(depth_image)
        depth_image_2 = np.uint8(255*(depth_image/4095))
        depth_image_2.resize((480,640))
        color_depth = cv2.applyColorMap(depth_image_2, cv2.COLORMAP_RAINBOW)
        
        #Draw rectangle at the center of depth image
        cv2.rectangle(color_depth,(315, 235),(325, 245),(0,0,0),2)      

        ir_image_2 = np.uint8(255*(ir_image/4095))
        ir_image_2.resize((480,640))
        ir_image_2 = cv2.applyColorMap(ir_image_2, cv2.COLORMAP_BONE)
        
        #Output images as .png 
        cv2.imwrite('ir.png', ir_image_2)
        cv2.imwrite('depth.png', color_depth)
        
        #Uncomment if running example on 96/Arrow board with HDMI to show stream data on monitor
        #cv2.imshow( "Depth", color_depth)
        #cv2.imshow( "IR", ir_image_2)  
        
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

        
