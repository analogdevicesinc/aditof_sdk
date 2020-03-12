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
import tof_device.tof_device as tofdev
import logging
import numpy as np
import pandas as pd
import os
import json

def mode_stop(dev):
    dev.writeAFEReg(0x4001, 0x0006)
    dev.writeAFEReg(0x7c22, 0x0004)
    
def mode_start(dev):
    dev.writeAFEReg(0x4001, 0x0007)
    dev.writeAFEReg(0x7c22, 0x0004)

def ld_control(dev, enable=0b0000):
    data = 0x0010 + enable    
    mode_stop(dev)
    dev.writeAFEReg(0xc084, data)
    dev.writeAFEReg(0xc085, data)
    mode_start(dev)
    
def get_ir_image(dev, buf):
    for x in range(buf + 1):
        dev.getImage()
    image = dev.getImage()
    image = np.resize(image, (960,640))
    irImage = pd.DataFrame(image[1::2, :])
    return irImage
    
def import_config_dict(path=''):
    config_dict = {}
    if(path is not None):
        with open(path) as f:
            config_dict = json.load(f)
    return config_dict
    
def crop_center(image, frame_width=640, frame_height=480, x=10, y=10):
    return image.iloc[int((frame_height-y)/2):int((frame_height+y)/2), int((frame_width-x)/2):int((frame_width+x)/2)]
    

def test_all_ld(dev, config_path = ''):
    logger = logging.getLogger(__name__)
    logger.info('Running LD Test')
    ret = []
    
    config_dict = import_config_dict(config_path)
    passed = True
    # Turn off all LDs and get value
    ld_control(dev, 0b0000)
    irImage = get_ir_image(dev, 3)
    cropped = crop_center(irImage)
    ret.append(np.average(cropped))
    if np.average(cropped) > config_dict['no_ld_max_threshold']:
        logger.error("LD TEST - No LD IR value: %d - Higher than threshold - FAILED", np.average)
        passed = False
    
    #LD1
    ld_control(dev, 0b0001)
    irImage = get_ir_image(dev, 3)
    cropped = crop_center(irImage)
    ret.append(np.average(cropped))
    if np.average(cropped) < config_dict['1_ld_min_threshold']:
        logger.error("LD TEST - LD1 IR value: %d - Lower than threshold - FAILED", np.average)
        passed = False
    #LD2   
    ld_control(dev, 0b0010)
    irImage = get_ir_image(dev, 3)
    cropped = crop_center(irImage)
    ret.append(np.average(cropped))
    if np.average(cropped) < config_dict['1_ld_min_threshold']:
        logger.error("LD TEST - LD2 IR value: %d - Lower than threshold - FAILED", np.average)
        passed = False
    
    #LD3    
    ld_control(dev, 0b0100)
    irImage = get_ir_image(dev, 3)
    cropped = crop_center(irImage)
    ret.append(np.average(cropped))
    if np.average(cropped) < config_dict['1_ld_min_threshold']:
        logger.error("LD TEST - LD3 IR value: %d - Lower than threshold - FAILED", np.average)
        passed = False
    
    #LD4    
    ld_control(dev, 0b1000)
    irImage = get_ir_image(dev, 3)
    cropped = crop_center(irImage)
    ret.append(np.average(cropped))
    if np.average(cropped) < config_dict['1_ld_min_threshold']:
        logger.error("LD TEST - LD4 IR value: %d - Lower than threshold - FAILED", np.average)
        passed = False
    
    if passed:
        logger.info("LD TEST - PASSED")
        return [1] + ret
    else:
        logger.info("LD TEST - FAILED")
        return [0] + ret
        
    
        
    
    
    
    