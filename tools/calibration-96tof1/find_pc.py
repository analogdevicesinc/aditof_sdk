import re
import aditofpython as tof
import core.frame as frame
import tof_calib.device as device
import numpy as np
import json
import os
import glob
import time
import logging
import logging.config
import sys
import click


log_dict = {}


def setup_logging():
    with open('logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)


def create_code_dict(text):
    reg = re.compile(r'([0-9a-f]{4}  [0-9a-f]{4})')
    rawinfo = re.findall(reg, text)
    code_dict = {}
    for x in rawinfo:
        s_line = re.split(r'\s', x)
        addr = int(s_line[0], 16)
        data = int(s_line[2], 16)
        code_dict[addr] = data
    return code_dict


def load_config_dict(config_json):
    sweep_config_dict = {}
    if(config_json is not None):
        with open(config_json) as f:
            sweep_config_dict = json.load(f)

    return sweep_config_dict


def Diff(list1, list2):
    return (list(set(list1)-set(list2)))


def get_target_pc(cam_handle, code_dict, pc, target_ir, max_pc, cfg, multiply_factor):
    switch_count = 0
    old_flag = 1
    flag = 1
    data = np.zeros(len(list(code_dict.keys())))
    data[:] = pc

    to_multiply_keys = [0x4110, 0x46aa, 0x4128, 0x46f2, 0x40c2]
    no_multiply_keys = Diff(list(code_dict.keys()), to_multiply_keys)

    data_simple = np.zeros(len(no_multiply_keys))
    data_simple[:] = pc

    data_mul = np.zeros(len(to_multiply_keys))
    data_mul[:] = pc * multiply_factor

    ir_crop = np.empty((cfg['frame_count'], cfg['window_x'], cfg['window_y']))

    while(switch_count != 3):

        # data_simple[:] = pc
        # device.write_AFE_reg(cam_handle, np.array(no_multiply_keys), data_simple)

        if multiply_factor != 1:  # avoid writing into registers coresponding to mid
            data_mul[:] = pc * multiply_factor
            device.write_AFE_reg(cam_handle, np.array(
                to_multiply_keys), data_mul)

        data[:] = pc
        device.write_AFE_reg(cam_handle, np.array(
            list(code_dict.keys())), data)

        for i in range(5):
            frame.dummy_read(cam_handle)
        for frame_ind in np.arange(0, cfg['frame_count']):
            ir_image = frame.get_ir_image_df(
                cam_handle, cfg['raw_frame_width'], cfg['raw_frame_height'])
            ir_crop[frame_ind] = frame.crop_center(
                ir_image, cfg['frame_width'], cfg['frame_height'], cfg['window_x'], cfg['window_y'])

        ir_val = np.average(ir_crop)

        print('PC:' + str(pc) + ' IR:' + str(ir_val))

        log_dict['results'].append({
            'PC': str(pc),
            'IR': str(ir_val)
        })

        if old_flag != flag:
            switch_count = switch_count + 1
        if switch_count == 2:
            break

        if pc >= max_pc:
            print('Could not achieve target IR')
            break

        if(ir_val < target_ir):
            if switch_count:
                pc = pc + 5
            else:
                pc = pc + 15
            flag = 1
        elif(ir_val > target_ir):
            if switch_count:
                pc = pc - 5
            else:
                pc = pc - 15
            flag = -1

    code_dict = dict.fromkeys(code_dict.keys(), pc)
    if multiply_factor != 1:  # check required in order to avoid KeyError in other modes than mid
        for addr in to_multiply_keys:
            code_dict[addr] = code_dict[addr] * multiply_factor
    return pc, code_dict


def tohex(val, nbits, format_string):
    return format((int(val) + (1 << nbits)) % (1 << nbits), format_string)


@click.command()
@click.argument('config-json', type=click.STRING)
def run_find_pc(config_json, **kwargs):
    logger = logging.getLogger(__name__)

    pc_config_dict = load_config_dict(config_json)

    log_dict['pc_config_dict'] = pc_config_dict
    sweep_config_dict = load_config_dict(
        pc_config_dict['firmware_config_json_path'])
    firmware_path = sweep_config_dict['firmware_path']
    mode = pc_config_dict['firmware_config_json_path'].split('/')[2]
    multiply_factor = 2 if mode == "M3W" else 1
    log_dict['mode'] = mode
    log_dict['results'] = []

    # Initialize tof class
    system = tof.System()

    ipString = ''
    # Get Camera and program firmware
    cam_handle = device.open_device2(system, ipString)
    firmware_path = sweep_config_dict['firmware_path']
    device.program_firmware2(cam_handle, firmware_path)

    # Open Repeat Number file and load PC
    lf_path = os.path.join(
        sweep_config_dict['firmware_path'], sweep_config_dict['repeat_num_filename'])
    file = open(lf_path)
    text = file.read()
    pc_dict = create_code_dict(text)

    target_ir = pc_config_dict['target_ir']
    start_pc = pc_config_dict['start_pc']
    max_pc = pc_config_dict['max_pc']

    pc, pc_dict = get_target_pc(
        cam_handle, pc_dict, start_pc, target_ir, max_pc, sweep_config_dict, multiply_factor)

    t = time.time()
    d = 0
    while d < sweep_config_dict['warmup_time']:
        d = time.time() - t

    pc, pc_dict = get_target_pc(
        cam_handle, pc_dict, pc, target_ir, max_pc, sweep_config_dict, multiply_factor)

    with open(pc_config_dict['out_file'], 'w') as outfile:
        json.dump(log_dict, outfile)

    # Create new lf file
    repeat_folder = 'r_lfs'
    files = glob.glob(os.path.join(
        sweep_config_dict['firmware_path'], repeat_folder, '9_*'))
    new_filename = '9_RepeatNumAddrList_' + str(len(files)) + '.lf'
    os.rename(lf_path, os.path.join(
        sweep_config_dict['firmware_path'], repeat_folder, new_filename))

    # Write to new pulse count to file
    file = open(lf_path, 'w')
    file.writelines('/*Pulsecount :' + str(pc) + '*/ \n')
    for key in pc_dict.keys():
        file.writelines(tohex(key, 16, '04x') + '  ' +
                        tohex(pc_dict[key], 16, '04x') + '\n')
    file.close()


if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_find_pc()
