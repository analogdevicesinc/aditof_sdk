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
import tof_calib.device as device
import click
import json
import logging
import logging.config
import uuid
import ipaddress

import cal_eeprom.cal_map as ce


def setup_logging():
    with open('logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)


@click.command()
@click.argument('config-json', type=click.STRING)
@click.option('--remote', type=click.STRING, help="To connect to a camera over network, specify the ip (e.g. '192.168.1.101')")
def run_eeprom_replace_cal(config_json, **kwargs):
    '''
    Replace the calibration data for a single mode in an ADI TOF module

    SWEEP_CONFIG_JSON: Specify the default sweep config json file. The default options can be overridden by explicitly specifying the options on the command line
    '''
    logger = logging.getLogger(__name__)

    config_dict = {}
    if(config_json is not None):
        with open(config_json) as f:
            config_dict = json.load(f)

    for key, value in kwargs.items():
        if value is not None:
            config_dict[key] = kwargs[key]
        if key in config_dict:
            if config_dict[key] is None:
                print(
                    'Need to specify %s either as a command line option or in the sweep config json file')
                raise SystemExit

    ipString = ''
    if 'remote' in config_dict:
        ip = ipaddress.ip_address(config_dict['remote'])
        print('Running script for a camera connected over Network at ip:', ip)
        ipString = config_dict['remote']

    # Retrieve parameters
    cal_map_path = config_dict['cal_map_path']
    firmware_path = config_dict['firmware_path']
    mode = config_dict['mode']

    # Open the ADI TOF Camera
    system = tof.System()
    cam_handle = device.open_device2(system, ipString)
    eeproms = []
    cam_handle.getEeproms(eeproms)
    eeprom = eeproms[0]

    # Read the cal map from the camera and store it locally
    cal = ce.cal_map()
    cal.read_eeprom_cal_map(eeprom)

    save_path = cal_map_path + "/eeprom_read_map.bin"
    logger.info(
        "Save the original cal map to .bin and .json files, path: %s", save_path)
    cal.save_cal_map(save_path)
    save_path = cal_map_path + "/eeprom_read_map.json"
    with open(save_path, 'w') as f:
        f.write(str(cal.calibration_map))
    f.close()

    # cal.display_cal_map()
    #input("Press Enter to continue...")

    # Replace a single mode's cal data
    logger.info("Replace mode %s cal data from path: %s", mode, save_path)
    cal.replace_eeprom_mode(mode, firmware_path +
                            "/linear_cal.json", firmware_path)
    save_path = cal_map_path + "/eeprom_read_map_modified.json"
    with open(save_path, 'w') as f:
        f.write(str(cal.calibration_map))
    f.close()

    # Write the cal map back to the camera's EEPROM
    cal.write_eeprom_cal_map(eeprom)
    logger.info("Write back to EEPROM complete")


if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_eeprom_replace_cal()
