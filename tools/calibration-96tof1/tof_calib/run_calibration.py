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
import pandas as pd
import os
import json
from natsort import natsorted, ns
from matplotlib import pyplot as plt
from . import gen_delays as gd
from . import sweep_calibration as sc
from . import intrinsic_calibration as ic
from . import write_to_lf_file as write_lf
from . import device as device
import core.frame as frame
import core.rail2 as rail
import logging


def run_sweep_calibration(cam_handle=None, cfg=None, firmware_path=None, min_dist=None, max_dist=None):

    frame_count = cfg['frame_count']
    window = {}
    window['X'] = cfg['window_x']
    window['Y'] = cfg['window_y']
    
    pulse_count = {}
    pulse_count['init'] = cfg['pulse_count_min']
    pulse_count['max'] = cfg['pulse_count_max']
    
    raw_frame = {}
    raw_frame['height'] = cfg['raw_frame_height']
    raw_frame['width'] = cfg['raw_frame_width']
    
    frame = {}
    frame['height'] = cfg['frame_height']
    frame['width'] = cfg['frame_width']
    
    repeat_num_file = os.path.join(firmware_path,cfg['repeat_num_filename'])
    hpt_data_file = os.path.join(firmware_path,cfg['hpt_data_filename'])
    data_file = os.path.join(firmware_path,cfg['data_filename'])
    seq_file = os.path.join(firmware_path,cfg['seq_info_filename'])

    if min_dist is None:
        min_dist = cfg['min_dist']
    if max_dist is None:
        max_dist = cfg['max_dist']
    dist_interval = cfg['dist_interval']
    dist_step = cfg['dist_step']
    min_delay = cfg['min_delay']
    max_delay = cfg['max_delay']
    target_distance = cfg['target_distance']
    depth_conv_gain = cfg['depth_conv_gain']
    sw_gain = cfg['sw_gain']
    sw_offset = cfg['sw_offset']

    if(cfg['round_dist_range_to_interval']):
        min_dist = int(min_dist/dist_interval)*dist_interval
        max_dist = round(max_dist/dist_interval)*dist_interval

    delay_dict = gd.generate_delays(hpt_data_file, data_file, min_delay, max_delay, seq_file)

    depth_stats_df = sc.pulse_sweep(delay_dict, min_dist, max_dist, dist_step, target_distance, \
                                    dist_interval, raw_frame, frame, frame_count, window, depth_conv_gain, sw_gain, sw_offset, cam_handle)

    xpower = sc.calc_xpower(cfg['xcorr'])

    #linear_offset_df = sc.calc_non_linear_offset(cfg['xcorr'], depth_stats_df)
    linear_offset_df, depth_stats_df = sc.calc_non_linear_offset2(cfg['xcorr'], depth_stats_df, sw_gain, sw_offset)


    return linear_offset_df, depth_stats_df

def initialize_rail(cfg):
    rail_offset = cfg['rail_offset']
    com_port = 'COM' + str(cfg['rail_port'])
    rail_handle = rail.Rail(com_port)
    rail_handle.writeRailOffset(rail_offset)
    rail_handle.moveRailMM(cfg['target_distance'])
    return rail_handle


def run_rail_calibration(cam_handle=None, rail_handle=None, cfg=None, firmware_path=None):
    frame_count = cfg['frame_count']
    window = {}
    window['X'] = cfg['window_x']
    window['Y'] = cfg['window_y']
    
    frame = {}
    frame['height'] = cfg['frame_height']
    frame['width'] = cfg['frame_width']
    
    raw_frame = {}
    raw_frame['height'] = cfg['raw_frame_height']
    raw_frame['width'] = cfg['raw_frame_width']
    
    repeat_num_file = os.path.join(firmware_path,cfg['repeat_num_filename'])
    hpt_data_file = os.path.join(firmware_path,cfg['hpt_data_filename'])
    data_file = os.path.join(firmware_path,cfg['data_filename'])
    seq_file = os.path.join(firmware_path,cfg['seq_info_filename'])

    min_dist = cfg['rail_min_dist']
    max_dist = cfg['rail_max_dist']
    dist_step = cfg['rail_dist_step']
    depth_conv_gain = cfg['depth_conv_gain']
    
    sw_gain = cfg['sw_gain']
    sw_offset = cfg['sw_offset']
    
    #Rail Calib call
    depth_stats_df = sc.rail_sweep(min_dist, max_dist, dist_step, raw_frame, frame, frame_count, window, depth_conv_gain, sw_gain, sw_offset, rail_handle, dev_handle)
    xpower = sc.calc_xpower(cfg['xcorr'])
    linear_offset_df = sc.calc_non_linear_offset(cfg['xcorr'], depth_stats_df)
    
    return linear_offset_df, depth_stats_df
 

def run_intrinsic_calibration(intrinsic_config_json, cam_handle, unique_id):

    logger = logging.getLogger(__name__)
    intrinsic_config_dict = {}
    if(intrinsic_config_dict is not None):
        with open(intrinsic_config_json) as f:
            intrinsic_config_dict = json.load(f)
            
    firmware_path = intrinsic_config_dict['firmware_path']    
    
    device.program_firmware2(cam_handle, firmware_path)

    logger.info("Running Intrinsic Calibration")
    
    config_path = intrinsic_config_dict['config_path']
    output_path = os.path.join(intrinsic_config_dict['results_path'], unique_id, intrinsic_config_dict['mode'])
    
    ic_c = ic.intrinsic_calibration()
    if intrinsic_config_dict['get_coordinates']:
        ic_c.get_coordinates(frame.get_ir_image(cam_handle))
        ic_c.output_coordinates(config_path)
    else:
        ic_c.load_coordinates(config_path)
        
    if intrinsic_config_dict['calibrate_intrinsic']:
        for i in range(5):
            frame.get_ir_image(cam_handle)
        num, _ , _  = ic_c.calibrate_intrinsic(frame.get_ir_image(cam_handle))
    
    if num >= (intrinsic_config_dict['min_checkerboards']):
        ic_c.output_intrinsic(output_path, intrinsic_config_dict['serial_number'])
    else:
        logger.error(str(num) + " checkerboards found, no params output")
        
    if intrinsic_config_dict['output_data']:
        ic_c.output_intrinsic_data(output_path)


def verify_sweep(cam_handle, cfg, firmware_path):

    device.program_firmware2(cam_handle, firmware_path)
    # run sweep again
    linear_offset_df, depth_stats_df = run_sweep_calibration(cam_handle=cam_handle, cfg=cfg, firmware_path=firmware_path, min_dist = cfg['verify_min_dist'], max_dist=cfg['verify_max_dist'])
    
    return linear_offset_df, depth_stats_df

def rail_verify(cam_handle, rail_handle, cfg, firmware_path):
    # program linear offset
    device.program_firmware2(cam_handle, firmware_path)
    
    # run sweep again
    linear_offset_df, depth_stats_df = run_rail_calibration(cam_handle=cam_handle, rail_handle=rail_handle, cfg=cfg, firmware_path=firmware_path)
    
    return linear_offset_df, depth_stats_df



