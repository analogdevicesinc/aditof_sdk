import aditofpython as tof
import device
import intrinsic_calibration as ic
import sys
import numpy as np
import pandas as pd
import os
from natsort import natsorted, ns
from matplotlib import pyplot as plt
import re
import json
import logging
import logging.config
from datetime import datetime
import time
import cv2

def setup_logging():
    with open('../logger.json', 'r') as f:
       config = json.load(f)
       logging.config.dictConfig(config)

def get_ir_image(cam_handle):
    
    image = device.get_depth_image(cam_handle)
    ir_image = image[480:, :]
    return ir_image
    

def run_intrinsic_calibration(intrinsic_config_json, **kwargs):
    
    intrinsic_config_dict = {}
    if(intrinsic_config_dict is not None):
        with open(intrinsic_config_json) as f:
            intrinsic_config_dict = json.load(f)
            
    firmware_path = intrinsic_config_dict['firmware_path']    
    file_list = natsorted(os.listdir("./"+firmware_path+"/"), alg=ns.IGNORECASE)[:14]
    print(file_list)
    
    #Initialize tof class
    system = tof.System()
    status = system.initialize()
    logger.info("System Initialize: " + str(status))

    # Get Camera and program firmware
    cam_handle = device.open_device2(system)
    firmware_path = intrinsic_config_dict['firmware_path']
    device.program_firmware2(cam_handle, firmware_path)
     
    device.write_AFE_reg(cam_handle, [0xC0A9], [0x0002])
    device.write_AFE_reg(cam_handle, [0x4001], [0x0006])
    device.write_AFE_reg(cam_handle, [0x7c22], [0x0004])
    device.write_AFE_reg(cam_handle, [0x4001], [0x0007])
    device.write_AFE_reg(cam_handle, [0x7c22], [0x0004])
    
    config_path = intrinsic_config_dict['config_path']
    output_path = intrinsic_config_dict['output_path']
    
    ic_c = ic.intrinsic_calibration()
    if intrinsic_config_dict['get_coordinates']:
        ic_c.get_coordinates(get_ir_image(cam_handle))
        ic_c.output_coordinates(config_path)
    else:
        ic_c.load_coordinates(config_path)
        
    if intrinsic_config_dict['calibrate_intrinsic']:
        num, _ , roi  = ic_c.calibrate_intrinsic(get_ir_image(cam_handle))
    
    if num >= (intrinsic_config_dict['min_checkerboards']):
        ic_c.output_intrinsic(output_path, intrinsic_config_dict['serial_number'])
    else:
        logger.error(str(num) + " checkerboards found, no params output")
        
    if intrinsic_config_dict['output_data']:
        ic_c.output_intrinsic_data(output_path)
        
if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_intrinsic_calibration(str(sys.argv[1]))
