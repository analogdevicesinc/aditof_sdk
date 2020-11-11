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
from matplotlib import pyplot as plt
import re
import seaborn as sns
import click
import json
import logging
import logging.config
import uuid
import cal_eeprom.cal_eeprom as cal_map
import tof_calib.run_calibration as calib
import tof_calib.device as device
import tof_calib.write_to_lf_file as write_lf
import tof_calib.save_results as save
import core.metrics_calculator as metcalc
import core.publish_results as publish
import core.frame as frame
import tests.test_ld as ld
import time
import glob

def clear_screen():
    try:
        os.system('clear')
    except:
        os.system('cls')
    

def get_user_input(retry=False):
    supported = ['id', 'focus', 'int', 'near','warm0', 'mid','warm1','far', 'save', 'exit']
    prompt = """
      SETUPS
    |------------|
    | id         | 
    |------------|
    | focus      |
    |------------|
    | int        |
    |------------|
    | near       |
    |------------|
    | warm0      |
    |------------|
    | mid        |
    |------------|
    | warm1      |
    |------------|
    | far        |
    |------------|
    | save       |
    |------------|
    | exit       |
    |------------|"""
    print("ADI CALIBRATION TOOL\n", prompt)
    if not retry:
        choice = input("\nSelect a setup > ")
    else:
        choice = input("\nPlease select a SUPPORTED setup > ")
    if choice not in supported:
        clear_screen()
        choice = get_user_input(retry=True)
    return choice


def setup_logging():
    with open('logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)
 

def generate_unique_id():
    id = uuid.uuid4()
    return id.hex


def split_unique_id(id, length):
    return [int(id[i:i+length],16) for i in range(0,len(id), length)]


def test_depth_error(metrics_df, cfg):
    
    if any(metrics_df['post_calibration_error_percent'] >= cfg['depth_perc_err_threshold']):
        logger.error("Depth Error GREATER than threshold - FAILED")
    else:
        logger.info("Depth error LESS than threshold - PASSED")


def write_verify_eeprom(path, dev_handle):
    ret = True
    cal1 = cal_map.cal_map()
    ret = parse_cal_mode_folder(path, cal1)
    with open (os.path.join(path,"eeprom_write_map.json"), 'w') as f:
        f.write(str(cal1.calibration_map))
    f.close()
    cal1.save_cal_map(os.path.join(path,"eeprom_write_map.bin"))
    logger.info("Writing EEPROM")
    cal1.write_eeprom_cal_map(dev_handle)

    cal2 = cal_map.cal_map()
    logger.info("Reading EEPROM for verification")
    cal2.read_eeprom_cal_map(dev_handle)

    with open (os.path.join(path,"eeprom_read_map.json"), 'w') as f:
        f.write(str(cal2.calibration_map))
    f.close()
    cal2.save_cal_map(os.path.join(path,"eeprom_read_map.bin"))

    if (cal_map.compare_map(cal1.calibration_map, cal2.calibration_map)!= True):
        ret =False
        logger.error("Failed to write to eeprom")
    else:
        logger.info("Write to EEPROM successful")

    return ret


def parse_cal_mode_folder(cal_folder_path, calmap):
    mode_dict = {'near' : 0, 'mid' : 1, 'far' : 2}
    ret = True
    if (os.path.isdir(cal_folder_path)):
        for mode, mode_value in mode_dict.items():
            path = os.path.join(cal_folder_path,mode)
            if (os.path.isdir(path)):
                path = os.path.join(path,'latest')
                if (os.path.isdir(path)):
                    if(os.path.exists(os.path.join(path,'linear_cal.json'))):
                        json_file = os.path.join(path,'linear_cal.json')
                        calmap.add_json_to_map((mode_dict[mode]*2+2), json_file)
                    else:
                        logger.error("Can't find the calibration result %s LINEAR CAL JSON File", path)

                    if(os.path.exists(os.path.join(path,'lf_files'))):
                        lf_path = os.path.join(path,'lf_files')
                        calmap.add_load_files_to_map((mode_dict[mode]*2+3), lf_path)
                        calmap.calibration_map[mode_dict[mode]*2+3]
                    else:
                        logger.error("Can't find the calibration result %s LOAD File folder", path)
                else:
                    logger.error("Can't find the calibration result %s folder", path)      
            else:
                    logger.error("Can't find the calibration result %s folder", path)
        intrinsic_path = os.path.join(cal_folder_path,'intrinsic')
        if(os.path.isdir(intrinsic_path)):
            intrinsic_file = os.path.join(intrinsic_path, 'camera_intrinsic.json')
            if(os.path.exists(intrinsic_file)):
                calmap.add_json_to_map(1, intrinsic_file)
            else:
                logger.error("Can't find the calibration result %s Camera Intrinsic file", intrinsic_file)                
        else:
            logger.error("Can't find the calibration result %s Camera Intrinsic folder", intrinsic_path)
    else:
        logger.error("Can't find the calibration result %s folder", path)
    return ret


# TODO: Figure out if the meas_depth calculation is working
def verify_depth(dev_handle, cfg, cfg_prev, target_dist, firmware_path, num_frames):

    device.program_firmware(dev_handle, firmware_path)
    # Get Parameters
    window = {}
    window['X'] = cfg['window_x']
    window['Y'] = cfg['window_y']
    raw_frame_dict = {}
    raw_frame_dict['height'] = cfg['raw_frame_height']
    raw_frame_dict['width'] = cfg['raw_frame_width']
    frame_dict = {}
    frame_dict['height'] = cfg['frame_height']
    frame_dict['width'] = cfg['frame_width']
    
    gain = cfg_prev['sw_gain']
    offset = cfg_prev['sw_offset']
    # Empty ndarray
    depthCrop = np.empty([1, num_frames, window['X'], window['Y']])
    frame.dummy_read(dev_handle)
    # Get image
    for i in range(num_frames):
       depthImage = frame.get_depth_image_df(dev_handle, raw_frame_dict['width'], raw_frame_dict['height'])
       depthCrop[0][i] = frame.crop_center(depthImage, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
    meas_depth = (np.median(depthCrop[0])*gain) + offset
    logger.info("Measured Depth for previous station calibrated firmware: %.2f" % meas_depth)
    #print(target_dist)
    depth_error = meas_depth-target_dist
    depth_error_perc = (depth_error/target_dist)*100
    logger.info("Depth error percent is:" + str(depth_error_perc))
    if depth_error_perc >= cfg['depth_perc_err_threshold']:
        logger.error("Depth Error GREATER than threshold - FAILED")
    else:
        logger.info("Depth error LESS than threshold - PASSED")


def run_ld_test(ld_config_json, sweep_config_json, dev_handle):
    
    sweep_config_dict = {}
    if(sweep_config_json is not None):
        with open(sweep_config_json) as f:
            sweep_config_dict = json.load(f)
            
    logger.info("Running LD Test")
            
    firmware_path = sweep_config_dict['firmware_path']
    device.program_firmware(dev_handle, firmware_path)
    ret = ld.test_all_ld(dev_handle, ld_config_json)
    
    if ret[0] == 1:
        logger.info("LD TEST VALUES: %d, %d, %d, %d, %d - PASSED", ret[1], ret[2], ret[3], ret[4], ret[5])
    else:
        logger.info("LD TEST VALUES: %d, %d, %d, %d, %d - FAILED", ret[1], ret[2], ret[3], ret[4], ret[5])

def write_to_cal_json(cal_json, cfg):
    cal_dict = {}
    if(cal_json is not None):
        with open(cal_json) as f:
            cal_dict = json.load(f)
    

    for key, nested_dict in cal_dict.items():
        for nested_key, value in nested_dict.items():
           if nested_key == '2':
               cal_dict[key][nested_key] = cfg['unique_id_list']
    return cal_dict


def run_all_calibration(sweep_config_json, dev_handle, unique_id, **kwargs):
    sweep_config_dict = {}
    if(sweep_config_json is not None):
        with open(sweep_config_json) as f:
            sweep_config_dict = json.load(f)

    for key,value in kwargs.items():
        if value is not None:
            sweep_config_dict[key] = kwargs[key]
        if key in sweep_config_dict:
            if sweep_config_dict[key] is None:
                print('Need to specify %s either as a command line option or in the sweep config json file')
                raise SystemExit
    
    sweep_config_dict['unique_id'] = unique_id 
    sweep_config_dict['unique_id_list'] = split_unique_id(sweep_config_dict['unique_id'], 4)    

    firmware_path = sweep_config_dict['firmware_path']
    
    verify_firmware_path = os.path.join(sweep_config_dict['results_path'], sweep_config_dict['unique_id'], sweep_config_dict['verify_mode'], 'latest/lf_files')

    device.program_firmware(dev_handle, firmware_path)
    logger.info("Running Sweep Calibration")
    
    latest, archive = save.make_results_dir(sweep_config_dict['results_path'], sweep_config_dict['unique_id'], sweep_config_dict['mode'])
    
    if sweep_config_dict['calibrate']:
        linear_offset_df, depth_stats_df = calib.run_sweep_calibration(dev_handle=dev_handle, cfg=sweep_config_dict, firmware_path=firmware_path)
        save.save_lf_files(latest, archive, firmware_path)
        write_lf.write_linear_offset_to_lf2(firmware_path, os.path.join(latest, 'lf_files'), sweep_config_dict, linear_offset_df)

    depth_stats_calibrated_df = pd.DataFrame()
    #if sweep_config_dict['verify_sweep']:
    #    logger.info("Verifying Sweep")
    #    _, depth_stats_calibrated_df = calib.verify_sweep(dev_handle, sweep_config_dict, os.path.join(latest, 'lf_files'))

    #logger.info("Calculating Metrics")
    metrics_df = pd.DataFrame()
    #if sweep_config_dict['verify_sweep'] and sweep_config_dict['calculate_metrics']:
    #    metrics_df = metcalc.calculate_metrics(depth_stats_df, depth_stats_calibrated_df)
    
    #test_depth_error(metrics_df, sweep_config_dict)

    logger.info("Verify depth error for previous station calibrated firmware")
    sweep_config_json_prev = glob.glob(os.path.join(verify_firmware_path, 'sweep*.json'))[0]
    sweep_config_dict_prev = {}
    if(sweep_config_json_prev is not None):
        with open(sweep_config_json_prev) as f:
            sweep_config_dict_prev = json.load(f)

    for key,value in kwargs.items():
        if value is not None:
            sweep_config_dict_prev[key] = kwargs[key]
        if key in sweep_config_dict_prev:
            if sweep_config_dict_prev[key] is None:
                print('Need to specify %s either as a command line option or in the sweep config json file')
                raise SystemExit
    
    verify_depth(dev_handle, sweep_config_dict, sweep_config_dict_prev, sweep_config_dict['target_distance'], verify_firmware_path, 50)

    logger.info("writing to calibration json files")
    cal_json = os.path.join(firmware_path,'linear_cal.json')
    output_cal_dict = write_to_cal_json(cal_json, sweep_config_dict)

    logger.info("All tests passed. Saving results...")
    save.save_results(latest, archive, depth_stats_df, depth_stats_calibrated_df, linear_offset_df, sweep_config_dict, firmware_path, metrics_df, output_cal_dict)

    return linear_offset_df, dev_handle, sweep_config_dict


def set_file_logging(unique_id):
   #FIXME: use results_path from json 
   logging_path = os.path.join('saved_results', unique_id)
   os.makedirs(logging_path, exist_ok=True)
   fh = logging.FileHandler(os.path.join('saved_results', unique_id, "calib.log")) 
   fh.setLevel(logging.DEBUG)
   formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
   fh.setFormatter(formatter)
   logger.addHandler(fh)

def save_ir_image(intrinsic_config_json, image_path, dev_handle):
    logger.info("Saving IR Image")
    intrinsic_config_dict = {}
    if(intrinsic_config_dict is not None):
        with open(intrinsic_config_json) as f:
            intrinsic_config_dict = json.load(f)
            
    firmware_path = intrinsic_config_dict['firmware_path']

    print(firmware_path)
    print(intrinsic_config_json)
    
    device.program_firmware(dev_handle, firmware_path)
    
    os.makedirs(image_path, exist_ok=True)
    focus = True
    dummy_count = 5
    while focus:
        for count in np.arange(dummy_count):
            dev_handle.getImage()
        ir_image = frame.get_ir_image(dev_handle)
        plt.figure('ir_image')
        plt.clf()
        plt.imshow(ir_image, cmap='gray')
        plt.draw()
        plt.pause(0.1)
        plt.savefig(os.path.join(image_path, 'ir_image.png'))
        publish.push_to_host(image_path)
        print("Press Enter to send new image \n")
        print("Enter -- done -- if focusing complete \n")
        choice = input()
        if choice == 'done':
            break
            
def warmup(sweep_config_dict, dev_handle, seconds):
    sweep_config_dict = {}
    if(sweep_config_json is not None):
        with open(sweep_config_json) as f:
            sweep_config_dict = json.load(f)
            
    firmware_path = sweep_config_dict['firmware_path']

    device.program_firmware(dev_handle, firmware_path)
    logger.info("Starting Warmup")
    start = time.time()
    end = time.time()
    while(end-start < seconds):
        end = time.time()
        if int(end)%10 == 0:
            logger.info("Time Left: %d seconds", seconds - (end - start))
            time.sleep(1)
            
    logger.info("Warmup Ended")

if __name__ == "__main__":
   # Clear screen and prompt
   clear_screen()
   unique_id = None
   while True:
       station = get_user_input()
       # Run clibration for selected setup
       if station == 'id':
           unique_id = input()
           unique_id = '1181' + unique_id
           # Setup logger
           setup_logging()
           logger = logging.getLogger(__name__)
           set_file_logging(unique_id)
           logger = logging.getLogger(__name__)
           dev_handle = device.open_device()
           logger.info("USER INPUT FOR CAMERA UNIQUE ID: %s", unique_id)
           logger.info("Camera ready for Calibration")
       if unique_id != None:   
           if station == 'focus':
               intrinsic_config = os.path.join('config/intrinsic_config.json')
               image_path = os.path.join('saved_results','images')
               save_ir_image(intrinsic_config, image_path, dev_handle)
           elif station == 'int':
               print("Running CAMERA INTRINSICS...\n")
               intrinsic_config = os.path.join('config/intrinsic_config.json')
               calib.run_intrinsic_calibration(intrinsic_config, dev_handle, unique_id)
           elif station == 'near':
               msg = "\nSelected: NEAR\n"
               sweep_config_json = os.path.join('config/BM_Kit/Near/sweep_config_near.json')
               ld_config_json = os.path.join('config/ld_config.json')
               print(msg)
               run_ld_test(ld_config_json, sweep_config_json, dev_handle)
               lin_offset_df, dev, cfg = run_all_calibration(sweep_config_json, dev_handle, unique_id)
           elif station == 'warm0':
               logger.info("warming up for mid mode")
               sweep_config_json = os.path.join('config/BM_Kit/M3W/sweep_config_m3w.json')
               warmup(sweep_config_json, dev_handle, 90)
           elif station == 'mid':
               msg = "\nSelected: MID\n"
               sweep_config_json = os.path.join('config/BM_Kit/M3W/sweep_config_m3w.json')
               print(msg)
               lin_offset_df, dev, cfg = run_all_calibration(sweep_config_json, dev_handle, unique_id)
           elif station == 'warm1':
               logger.info("warming up for far mode")
               sweep_config_json = os.path.join('config/BM_Kit/Far/sweep_config_far.json')
               warmup(sweep_config_json, dev_handle, 150)
           elif station == 'far':
               msg = "\nSelected: FAR\n"
               sweep_config_json = os.path.join('config/BM_Kit/Far/sweep_config_far.json')
               print(msg)
               lin_offset_df, dev, cfg = run_all_calibration(sweep_config_json, dev_handle, unique_id)
           elif station == 'save':
               sweep_config_json = os.path.join('config/BM_Kit/Far/sweep_config_far.json')
               camera_results_path = save.get_results_path(sweep_config_json, unique_id)
               logger.info('camera results path %s', camera_results_path)
               write_verify_eeprom(camera_results_path, dev_handle)
               publish.push_to_host(camera_results_path)
               logger.info("Results written to host: Camera Unique ID: %s", unique_id)
       else:
           logger.error("UNIQUE ID INPUT REQUIRED FIRST")
            
       if station == 'exit':
           logger.info("Results written to host: Camera Unique ID: %s", unique_id)
           break
       else:
           print("Unsupported option... Aborting...\n")


