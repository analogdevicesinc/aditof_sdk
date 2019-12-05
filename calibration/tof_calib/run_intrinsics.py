import tof_device.tof_device as tofdev
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

def setup_logging():
    with open('../logger.json', 'r') as f:
       config = json.load(f)
       logging.config.dictConfig(config)

def get_ir_image(dev):
    
    image = dev.getImage()
    image = np.resize(image, (960,640))
    irImage = image[1::2, :]
    imax = np.amax(irImage)
    irImage = np.uint8(255*(irImage/imax))
    irImage.resize((480,640))
    return irImage
    

def run_intrinsic_calibration(intrinsic_config_json, **kwargs):
    
    intrinsic_config_dict = {}
    if(intrinsic_config_dict is not None):
        with open(intrinsic_config_json) as f:
            intrinsic_config_dict = json.load(f)
            
    firmware_path = intrinsic_config_dict['firmware_path']    
    file_list = natsorted(os.listdir("./"+firmware_path+"/"), alg=ns.IGNORECASE)[:14]
    print(file_list)
    
    dev_handle = tofdev.tof_device()
    ret = dev_handle.openDevice(2,-1)
    if ret != -1:
        logger.info("Device Opened")
    else:
        logger.error("Can't open device")
        raise SystemExit
        
    for file_name in file_list:
        if file_name.endswith(".lf"):
            dev_handle.parseCode("./"+firmware_path+"/"+file_name)
            
    dev_handle.program()
    dev_handle.writeAFEReg(0xC0A9, 2) # Set to ISA mode

    dev_handle.writeAFEReg(0x4001, 0x0006)
    dev_handle.writeAFEReg(0x7c22, 0x0004)
    dev_handle.writeAFEReg(0x4001, 0x0007)
    dev_handle.writeAFEReg(0x7c22, 0x0004)# Mode stop and start sequence 
    
    config_path = intrinsic_config_dict['config_path']
    output_path = intrinsic_config_dict['output_path']
    
    ic_c = ic.intrinsic_calibration()
    if intrinsic_config_dict['get_coordinates']:
        ic_c.get_coordinates(get_ir_image(dev_handle))
        ic_c.output_coordinates(config_path)
    else:
        ic_c.load_coordinates(config_path)
        
    if intrinsic_config_dict['calibrate_intrinsic']:
        num, _ , _  = ic_c.calibrate_intrinsic(get_ir_image(dev_handle))
    
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
