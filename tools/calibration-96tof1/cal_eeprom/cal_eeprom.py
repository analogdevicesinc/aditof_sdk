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
'''
===================
Calibration Map
===================

This module contains functions for generating calibration map for storing in EEPROM, storing in file, reading it back from file and parsing the map. 
'''

from collections import namedtuple
import struct
import sys
from natsort import natsorted, ns
import re
# import os
from cal_map import cal_map
import json
import aditofpython as tof
import logging
import logging.config
import numpy as np
from cal_map_consts import *


def setup_logging():
    with open('./../logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)


def flatten_cal_map(cal):
    lst = []
    for key, list_params in cal.items():
        size, nested_dict = list_params
        lst.append(key)
        lst.append(size)
        for nested_key, nested_value in nested_dict.items():
            param_size, param_value = nested_value
            lst.append(param_size)
            for i in range(int(param_size/4)):
                lst.append(param_value[i])
    return lst


def compare_map(cal1, cal2):
    lst1 = flatten_cal_map(cal1)
    lst2 = flatten_cal_map(cal2)
    ret = True
    for i in range(len(lst1)):
        if(lst1[i]-lst2[i] > 0.2):
            #print(lst1[1], lst2[2])
            ret = False
    return ret


'''
Test function to:
    Generate default map
    Write map to binary file
    Read map back from binary file
    Add load files to map
    Display the map
---------------------------------
'''


def test_cal_eeprom():
    cal1 = cal_map()
    cal1.init_default_cal_map()
    print("Calibration map written to Bin File")
    cal1.display_cal_map()
    cal1.save_cal_map("calibration_map.bin")

    cal2 = cal_map()
    cal2.read_cal_map("calibration_map.bin")
    print("\n\nCalibration map read back from Bin File")
    cal2.display_cal_map()
    input("Press Enter to continue...")

    cal2.add_json_to_map(NEAR_CAL, "./linear_cal.json")
    print("\n\nCalibration map after adding linear_cal.json to map")
    cal2.display_cal_map()
    input("Press Enter to continue...")

    cal2.add_load_files_to_map(NEAR_LF, "../config/ADDI9043/")
    print("\n\nCalibration map after adding load files to map")
    cal2.display_cal_map()
    print("\n\nSaving to 'calibration_map.bin'")
    cal2.save_cal_map("caibration_map.bin")
    input("Press Enter to continue...")

    #cal2.add_linear_correct_offset(NEAR_CAL, "../saved_results/latest/linear_offset.csv")
    # cal2.display_cal_map()
    # cal2.save_cal_map()

    # Open the ADI TOF Camera
    system = tof.System()
    status = system.initialize()
    print("system.initialize()", status)
    cam_handle = device.open_device2(system)
    eeproms = []
    cam_handle.getEeproms(eeproms)
    eeprom = eeproms[0]

    print("\n\nWriting to EEPROM")
    cal2.write_eeprom_cal_map(eeprom)

    print("\n\nReading from EEPROM")
    cal3 = cal_map()
    cal3.read_eeprom_cal_map(eeprom)
    cal3.save_cal_map("eeprom_read_map.bin")
    with open("eeprom_read_map.json", 'w') as f:
        f.write(str(cal3.calibration_map))
    f.close()
    cal3.display_cal_map()
    input("Press Enter to continue...")

    cal3.replace_eeprom_mode(
        'near', "./config/BM_Kit/Near/linear_cal.json", "./config/BM_Kit/Near/")
    with open("eeprom_read_map_modified.json", 'w') as f:
        f.write(str(cal1.calibration_map))
    f.close()


'''
Start point of program
---------------------------------
'''
if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    test_cal_eeprom()
