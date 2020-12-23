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
import pandas as pd
import os
import tof_calib.run_calibration as calib
import tof_calib.write_to_lf_file as write_lf
import tof_calib.save_results as save
import tof_calib.device as device
import core.metrics_calculator as metcalc
import cal_eeprom.cal_map as cal_map
import tof_calib.sweep_calibration as sc
# import core.report_generator as reportgen
import click
import json
import logging
import logging.config
import uuid
import time
import ipaddress


def setup_logging():
    with open('logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)


def generate_unique_id():
    id = uuid.uuid4()
    return id.hex


def split_unique_id(id, length):
    return [int(id[i:i+length], 16) for i in range(0, len(id), length)]


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


def write_verify_eeprom(path, dev_handle):
    ret = True
    cal1 = cal_map.cal_map()
    ret = parse_cal_mode_folder(path, cal1)
    with open(os.path.join(path, "eeprom_write_map.json"), 'w') as f:
        f.write(str(cal1.calibration_map))
    f.close()
    cal1.save_cal_map(os.path.join(path, "eeprom_write_map.bin"))
    logger.info("Writing EEPROM")
    cal1.write_eeprom_cal_map(dev_handle)

    cal2 = cal_map.cal_map()
    logger.info("Reading EEPROM for verification")
    cal2.read_eeprom_cal_map(dev_handle)

    with open(os.path.join(path, "eeprom_read_map.json"), 'w') as f:
        f.write(str(cal2.calibration_map))
    f.close()
    cal2.save_cal_map(os.path.join(path, "eeprom_read_map.bin"))

    if (cal_map.compare_map(cal1.calibration_map, cal2.calibration_map) != True):
        ret = False
        logger.error("Failed to write to eeprom")
    else:
        logger.info("Write to EEPROM successful")

    return ret


def parse_cal_mode_folder(cal_folder_path, calmap):
    mode_dict = {'near': 0, 'mid': 1, 'far': 2}
    ret = True
    if (os.path.isdir(cal_folder_path)):
        for mode, mode_value in mode_dict.items():
            path = os.path.join(cal_folder_path, mode)
            if (os.path.isdir(path)):
                path = os.path.join(path, 'latest')
                if (os.path.isdir(path)):
                    if(os.path.exists(os.path.join(path, 'linear_cal.json'))):
                        json_file = os.path.join(path, 'linear_cal.json')
                        calmap.add_json_to_map(
                            (mode_dict[mode]*2+2), json_file)
                    else:
                        logger.error(
                            "Can't find the calibration result %s LINEAR CAL JSON File", path)

                    if(os.path.exists(os.path.join(path, 'lf_files'))):
                        lf_path = os.path.join(path, 'lf_files')
                        calmap.add_load_files_to_map(
                            (mode_dict[mode]*2+3), lf_path)
                        calmap.calibration_map[mode_dict[mode]*2+3]
                    else:
                        logger.error(
                            "Can't find the calibration result %s LOAD File folder", path)
                else:
                    logger.error(
                        "Can't find the calibration result %s folder", path)
            else:
                logger.error(
                    "Can't find the calibration result %s folder", path)
        intrinsic_path = os.path.join(cal_folder_path, 'intrinsic')
        if(os.path.isdir(intrinsic_path)):
            intrinsic_file = os.path.join(
                intrinsic_path, 'camera_intrinsic.json')
            if(os.path.exists(intrinsic_file)):
                calmap.add_json_to_map(1, intrinsic_file)
            else:
                logger.error(
                    "Can't find the calibration result %s Camera Intrinsic file", intrinsic_file)
        else:
            logger.error(
                "Can't find the calibration result %s Camera Intrinsic folder", intrinsic_path)
    else:
        logger.error("Can't find the calibration result %s folder", path)
    return ret


@click.command()
@click.argument('sweep-config-json', type=click.STRING)
@click.option('--remote', type=click.STRING, help="To connect to a camera over network, specify the ip (e.g. '192.168.1.101')")
@click.option('--firmware-path', type=click.Path(exists=True), help='Path to the firmware files (bin and lf)')
@click.option('--target-distance', type=click.FloatRange(0, 5000), help='Distance of the target board from the camera in millimeters')
@click.option('--camera-id', help='Unique camera identifier for the camera being calibrated')
@click.option('--min-dist', type=click.FLOAT, help='Minimum distance for Calibration in millimeters')
@click.option('--max-dist', type=click.FLOAT, help='Maximum distance for Calibration in millimeters')
@click.option('--min-delay', help='Minimum delay for Calibration')
@click.option('--max-delay', help='Maximum delay for Calibration')
@click.option('--frame-count', help='Number of frames to capture to calculate depth statistics per a given distance')
@click.option('--window-x', help='Region of Interest for depth statistics - value in x  direction - horizontal lenght')
@click.option('--window-y', type=click.INT, help='Region of Interest for depth statistics - value in y  direction - vertical lenght')
@click.option('--pulse-count-min', type=click.INT, help='Minimum Pulse count')
@click.option('--pulse-count-max', type=click.INT, help='Maximum Pulse count')
@click.option('--frame-height', type=click.INT, help='Frame Height')
@click.option('--frame-width', type=click.INT, help='Frame Width')
@click.option('--dist-interval', type=click.INT, help='Distance between each delay')
@click.option('--dist-step', type=click.FLOAT, help='Distance step size between each sweep step ')
@click.option('--depth-conv-gain', type=click.INT, help='Depth Conversion Gain')
@click.option('--results-path', type=click.STRING, default='saved_results')
@click.option('--verify-sweep')
@click.option('--calculate-metrics', help='Calculate calibration metrics')
@click.option('--show-report', help='Show calibration report')
def run_all_calibration(sweep_config_json, **kwargs):
    '''
    A Python based command line utility to run calibration for ADI TOF modules

    SWEEP_CONFIG_JSON: Specify the default sweep config json file. The default options can be overriden by explictly specifying the options on the command line
    '''

    sweep_config_dict = {}
    if(sweep_config_json is not None):
        with open(sweep_config_json) as f:
            sweep_config_dict = json.load(f)

    for key, value in kwargs.items():
        if value is not None:
            sweep_config_dict[key] = kwargs[key]
        if key in sweep_config_dict:
            if sweep_config_dict[key] is None:
                print(
                    'Need to specify %s either as a command line option or in the sweep config json file')
                raise SystemExit

    if sweep_config_dict['unique_id'] == None:
        unique_id = generate_unique_id()
        sweep_config_dict['unique_id'] = unique_id
    sweep_config_dict['unique_id_list'] = split_unique_id(
        sweep_config_dict['unique_id'], 4)

    ipString = ''
    if 'remote' in sweep_config_dict:
        ip = ipaddress.ip_address(sweep_config_dict['remote'])
        print('Running script for a camera connected over Network at ip:', ip)
        ipString = sweep_config_dict['remote']

    # Initialize tof class
    system = tof.System()

    # Get Camera and program firmware
    cam_handle = device.open_device2(system, ipString)
    firmware_path = sweep_config_dict['firmware_path']
    device.program_firmware2(cam_handle, firmware_path)

    logger.info("Running Calibration")
    # Initialize Dataframes
    depth_stats_calibrated_df = pd.DataFrame([], columns=['NO DATA'])
    depth_stats_df = pd.DataFrame([], columns=['NO DATA'])
    linear_offset_df = pd.DataFrame([], columns=['NO DATA'])
    metrics_df = pd.DataFrame([], columns=['NO DATA'])

    # If using rail create handle and store handle in cfg
    rail_handle = None
    if (sweep_config_dict['calib_type'] == 'Rail' or sweep_config_dict['verification_type'] == 'Rail'):
        rail_handle = calib.initialize_rail(sweep_config_dict)

    latest, archive = save.make_results_dir(
        sweep_config_dict['results_path'], sweep_config_dict['unique_id'], mode=sweep_config_dict['mode'])

    t = time.time()
    d = 0
    while d < sweep_config_dict['warmup_time']:
        d = time.time() - t

    if sweep_config_dict['calibrate']:
        if sweep_config_dict['calib_type'] == 'Sweep':
            linear_offset_df, depth_stats_df = calib.run_sweep_calibration(
                cam_handle=cam_handle, cfg=sweep_config_dict, firmware_path=firmware_path)
        elif sweep_config_dict['calib_type'] == 'Rail':
            logger.info("Running Rail Calibration")
            # Run Rail calibration and store lf files
            linear_offset_df, depth_stats_df = calib.run_rail_calibration(
                cam_handle=cam_handle, rail_handle=rail_handle, cfg=sweep_config_dict, firmware_path=firmware_path)

        save.save_lf_files(latest, archive, firmware_path)
        write_lf.write_linear_offset_to_lf2(firmware_path, os.path.join(
            latest, 'lf_files'), sweep_config_dict, linear_offset_df)

    depth_stats_calibrated_df = pd.DataFrame()
    if sweep_config_dict['verify_sweep']:
        if sweep_config_dict['verification_type'] == 'Sweep':
            logger.info("Verifying Sweep")
            _, depth_stats_calibrated_df = calib.verify_sweep(
                cam_handle, sweep_config_dict, os.path.join(latest, 'lf_files'))

        elif sweep_config_dict['verification_type'] == 'Rail':
            logger.info("Verifying Sweeap")
            device.program_firmware2(
                cam_handle, os.path.join(latest, 'lf_files'))
            #_, depth_stats_calibrated_df = calib.rail_verify(cam_handle, rail_handle, sweep_config_dict, os.path.join(latest, 'lf_files'))
            cfg = sweep_config_dict
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

            repeat_num_file = os.path.join(
                firmware_path, cfg['repeat_num_filename'])
            hpt_data_file = os.path.join(
                firmware_path, cfg['hpt_data_filename'])
            data_file = os.path.join(firmware_path, cfg['data_filename'])
            seq_file = os.path.join(firmware_path, cfg['seq_info_filename'])

            min_dist = cfg['rail_min_dist']
            max_dist = cfg['rail_max_dist']
            dist_step = cfg['rail_dist_step']
            depth_conv_gain = cfg['depth_conv_gain']

            sw_gain = cfg['sw_gain']
            sw_offset = cfg['sw_offset']
            depth_stats_calibrated_df = sc.rail_stats(
                min_dist, max_dist, dist_step, raw_frame, frame, frame_count, window, sw_gain, sw_offset, rail_handle, cam_handle)

    logger.info("Calculating Metrics")
    metrics_df = pd.DataFrame()
    # if sweep_config_dict['verify_sweep'] and sweep_config_dict['calculate_metrics']:
    #    metrics_df = metcalc.calculate_metrics(depth_stats_df, depth_stats_calibrated_df)

    logger.info("writing to calibration json files")
    cal_json = os.path.join(firmware_path, 'linear_cal.json')
    output_cal_dict = write_to_cal_json(cal_json, sweep_config_dict)

    save.save_results(latest, archive, depth_stats_df, depth_stats_calibrated_df,
                      linear_offset_df, sweep_config_dict, firmware_path, metrics_df, output_cal_dict)

    camera_results_path = save.get_results_path(
        sweep_config_json, sweep_config_dict['unique_id'])
    logger.info('camera results path %s', camera_results_path)
    return linear_offset_df


if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_all_calibration()
