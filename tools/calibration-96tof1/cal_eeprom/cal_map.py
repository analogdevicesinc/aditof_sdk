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

import pandas as pd
import numpy as np
import os
import re
from natsort import natsorted, ns
import sys
import struct
from collections import namedtuple
import firmware_gen as lf
from cal_map_consts import *
import logging
import json
'''
Class for managing the calibration map
Consist functions to:
    generate calibration map
    store calibration map binary to file
    read calibration map from binary file
    parse binary back to calibration map
    display calibration map
---------------------------------
'''


def setup_logging():
    with open('logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)


logger = logging.getLogger(__name__)


class cal_map(object):

    def __init__(self):
        setup_logging()
        self.calibration_map = {}
        header_packet = {
            TOTAL_SIZE: self.param_struct([8]),
            CHECKSUM: self.param_struct([8])
        }
        self.calibration_map = {
            HEADER: [self.get_packet_size(header_packet), header_packet],
        }

    # calculates size of value and returns list[size, value]
    def param_struct(self, param_value):
        size = len(param_value) * 4  # len * 4(each element is float)
        param_value = [int(size), [float(i) for i in param_value]]
        return param_value

    # calculates and returns size of packet
    def get_packet_size(self, packet):
        packet_size = 0
        for nested_key, nested_value in packet.items():
            param_size, param_value = nested_value
            # added size 8 for key and size of each parameter
            packet_size = packet_size + param_size + 8
        return int(packet_size)

    # calculates and returns size of map
    def get_map_size(self):
        map_size = 0
        for key, list_params in self.calibration_map.items():
            size, nested_dict = list_params
            map_size = map_size + size
            # Size of each key(4)  and packet size(4) is added(4+4=8)
            map_size = map_size + 8
        return map_size

    def update_packet_checksum(self, packet):
        checksum = 0
        for nested_key, nested_value in packet.items():
            param_size, param_value = nested_value
            for i in range(int(param_size/4)):
                checksum = int(checksum) ^ int(param_value[i])
        packet[CHECKSUM] = self.param_struct([checksum])

    def update_map_header(self):
        # Update Header Total Size
        total_size = self.get_map_size()
        self.calibration_map[HEADER][VALUE][TOTAL_SIZE] = self.param_struct([
                                                                            total_size])
        # Update Header Checksum
        self.update_packet_checksum(self.calibration_map[HEADER][VALUE])

    # Generates Default Dictionary
    def init_default_cal_map(self):
        header_packet = {
            EEPROM_VERSION: self.param_struct([0]),
            TOTAL_SIZE: self.param_struct([1000]),
            NUMBER_OF_MODES: self.param_struct([3]),
        }
        self.update_packet_checksum(header_packet)

        camera_intrinsic_packet = {
            EEPROM_VERSION: self.param_struct([0]),
            CAL_SER_NUM: self.param_struct([0]),
            CAL_DATE: self.param_struct([12042019]),
            INTRINSIC: self.param_struct([0, 0, 0, 0, 0, 0, 0, 0, 0])
        }
        self.update_packet_checksum(camera_intrinsic_packet)

        self.calibration_map = {
            HEADER: [self.get_packet_size(header_packet), header_packet],
            CAMERA_INTRINSIC: [self.get_packet_size(
                camera_intrinsic_packet), camera_intrinsic_packet]
        }
        # Update Header
        self.update_map_header()

    # Parses through dictionary and prints the key and value

    def display_cal_map(self):
        # Printing just the value of Calibration Dictionary
        for key, list_params in self.calibration_map.items():
            # print the primary key (for Packet Type)
            print("Packet Key: ", (key), end="")
            size, nested_dict = list_params
            print("\tPacket Size: ", size)  # print the size of pimary packet
            for nested_key, nested_value in nested_dict.items():
                # print the nested key (Parameter key)
                print("\tParam Key: ", nested_key, end="")
                param_size, param_value = nested_value
                # print the size of Param
                print("\tParam Size: ", param_size, end="")
                value = []
                for i in range(int(param_size/4)):
                    value.append(param_value[i])
                print("\tParam Value: ", value)  # print the value of Param

    # Generates the binary file for writing to EEPROM

    def save_cal_map(self, filename):
        # writing float values
        f = open(filename, "wb")
        f.write(struct.pack('<f', self.get_map_size()))
        for key, list_params in self.calibration_map.items():
            # write the primary key (for Packet Type)
            f.write(struct.pack('<f', key))
            struct.pack('<f', key)
            size, nested_dict = list_params
            # write the size of pimary packet size
            f.write(struct.pack('<f', size))
            for nested_key, nested_value in nested_dict.items():
                # write the nested key (Parameter key)
                f.write(struct.pack('<f', nested_key))
                param_size, param_value = nested_value
                # write the size of Param
                f.write(struct.pack('<f', param_size))
                for i in range(int(param_size/4)):
                    # write the value of Param
                    f.write(struct.pack('<f', param_value[i]))
        f.close()

    '''Reads the binary file and parses it back to map,
    replaces the value if already exist'''

    def read_cal_map(self, filename):
        # open the file
        with open(filename, "rb") as f:
            size = f.read(4)
            while True:
                key = f.read(4)
                if not key:
                    break
                key = struct.unpack('<f', key)
                key = int(key[0])
                sub_packet_size = struct.unpack('<f', f.read(4))
                sub_packet_size = int(sub_packet_size[0])
                sub_packet_map = {}
                i = 0
                while i < (sub_packet_size/4):  # 4:size of float
                    sub_packet_value = struct.unpack('<f', f.read(4))
                    sub_packet_value = int(sub_packet_value[0])
                    i = i + 1
                    parameter_key = sub_packet_value

                    sub_packet_value = struct.unpack('<f', f.read(4))
                    sub_packet_value = int(sub_packet_value[0])
                    i = i + 1
                    parameter_size = sub_packet_value

                    number_of_elements = int(
                        parameter_size/4)  # 4:size of float

                    value = []
                    for j in range(number_of_elements):
                        sub_packet_value = struct.unpack('<f', f.read(4))
                        value.append(sub_packet_value[0])
                        i = i + 1
                    sub_packet_map.update(
                        {parameter_key: [parameter_size, value]})
                self.calibration_map[key] = [sub_packet_size, sub_packet_map]
        f.close()

    # Add Load files to map, if existing map consist load files, it overwrites it, otherwise adds it
    def add_load_files_to_map(self, packet_type, lf_path):
        lf_map = {}
        lf_list = []
        file_list = natsorted(os.listdir(
            "./"+lf_path+"/"), alg=ns.IGNORECASE)[:13]
        logger.debug(file_list)
        for file_name in file_list:
            if file_name.endswith(".lf"):
                addr, data, mode_locations = lf.extract_code_block(
                    "./"+lf_path+"/"+file_name)
                for i in range(len(addr)):
                    lf_list.append(addr[i])
                    lf_list.append(data[i])
                logger.debug("Parsed File", file_name,
                             "\n", lf_list)

        lf_map[ADDR_DATA_LIST] = self.param_struct(lf_list)
        self.update_packet_checksum(lf_map)
        self.calibration_map[packet_type] = [
            self.get_packet_size(lf_map), lf_map]
        # Update Header
        self.update_map_header()

    def add_linear_offset_csv_to_map(self, packet_type, linear_offset_csv_file):
        linear_df = pd.read_csv(linear_offset_csv_file)
        linear_correct_offset_list = (linear_df.to_dict(
            orient='list')["reg_offset_value_hex"])
        linear_correct_xpwr_list = (
            linear_df.to_dict(orient='list')["xcorr"][1:])
        linear_map = {}
        linear_map[LINEAR_CORRECT_OFFSET] = self.param_struct(
            [int(i, 16) for i in linear_correct_offset_list])
        linear_map[LINEAR_CORRECT_XPWR] = self.param_struct(
            linear_correct_xpwr_list)
        self.calibration_map[packet_type] = [
            self.get_packet_size(linear_map), linear_map]
        # Update Header
        self. update_map_header()

    def add_json_to_map(self, packet_type, json_file):
        with open(json_file, 'r') as f:
            json_read = json.load(f)
            json_map = {}
            for key, value in json_read.items():
                for sub_key, sub_value in json_read[key].items():
                    if(type(sub_value) is list):
                        json_map[int(sub_key)] = self.param_struct(sub_value)
                    else:
                        json_map[int(sub_key)] = self.param_struct([sub_value])
            self.update_packet_checksum(json_map)
            self.calibration_map[packet_type] = [
                self.get_packet_size(json_map), json_map]
            self.update_map_header()

    # Function to replace calibration mode block
    def replace_eeprom_mode(self, mode, linear_cal_json_file, load_file_path):
        self.add_json_to_map((get_cal_key(mode)), linear_cal_json_file)
        self.add_load_files_to_map((get_lf_key(mode)), load_file_path)

    def write_eeprom_cal_map(self, eeprom):
        logger.debug("\n\nWriting EEPROM")
        eeprom_write_bytearray = bytes()
        for key, list_params in self.calibration_map.items():
            eeprom_write_bytearray = eeprom_write_bytearray + \
                (struct.pack('<f', key))  # write the primary key (for Packet Type)
            struct.pack('<f', key)
            size, nested_dict = list_params
            eeprom_write_bytearray = eeprom_write_bytearray + \
                (struct.pack('<f', size))  # write the size of pimary packet size
            for nested_key, nested_value in nested_dict.items():
                eeprom_write_bytearray = eeprom_write_bytearray + \
                    (struct.pack('<f', nested_key)
                     )  # write the nested key (Parameter key)
                param_size, param_value = nested_value
                eeprom_write_bytearray = eeprom_write_bytearray + \
                    (struct.pack('<f', param_size))  # write the size of Param
                for i in range(int(param_size/4)):
                    eeprom_write_bytearray = eeprom_write_bytearray + \
                        (struct.pack(
                            '<f', param_value[i]))  # write the value of Param
        eeprom_map_size = [self.get_map_size()]

        eeprom_write_list = []
        size = eeprom_write_bytearray.__len__()
        for index in range(0, size):
            eeprom_write_list.append(eeprom_write_bytearray[index])
        logger.debug("EEPROM WRITE List\n", eeprom_write_list)

        size_list = []
        size_byte = bytes()
        size_byte = struct.pack('<f', size)
        for index in range(0, 4):
            size_list.append(size_byte[index])
        eeprom.write(int(0), np.array(size_list, dtype='uint8'), 4)
        eeprom.write(int(4), np.array(eeprom_write_list,
                                      dtype='uint8'), eeprom_write_list.__len__())

    def read_eeprom_cal_map(self, eeprom):
        logger.debug("Reading EEPROM")
        data_array = np.zeros(4, dtype='uint8')
        eeprom.read(int(0), data_array, 4)
        read_size = struct.unpack('<f', data_array)
        logger.debug("Read Size", read_size)

        data_array = np.zeros(int(read_size[0]), dtype='uint8')
        eeprom.read(int(4), data_array, int(read_size[0]))
        r_b = data_array.tobytes()

        j = 0
        while j < r_b.__len__():
            key = r_b[j:j+4]
            j = j+4
            if not key:
                break
            key = struct.unpack('<f', key)
            key = int(key[0])
            logger.debug("Primary Key", key)
            sub_packet_size = struct.unpack('<f', r_b[j:j+4])
            j = j+4
            sub_packet_size = int(sub_packet_size[0])
            logger.debug("Sub Size", sub_packet_size)
            sub_packet_map = {}
            i = 0
            while i < (sub_packet_size/4):  # 4:size of float
                sub_packet_value = struct.unpack('<f', r_b[j:j+4])
                j = j+4
                sub_packet_value = int(sub_packet_value[0])
                i = i + 1
                parameter_key = sub_packet_value
                logger.debug("Param Key", parameter_key)

                sub_packet_value = struct.unpack('<f', r_b[j:j+4])
                j = j+4
                sub_packet_value = int(sub_packet_value[0])
                i = i + 1
                parameter_size = sub_packet_value
                logger.debug("Param Size", parameter_size)

                number_of_elements = int(parameter_size/4)  # 4:size of float
                logger.debug("Number of elements", number_of_elements)
                value = []
                for k in range(number_of_elements):
                    logger.debug(r_b[j:j+4])
                    sub_packet_value = struct.unpack('<f', r_b[j:j+4])
                    j = j+4
                    value.append(sub_packet_value[0])
                    i = i + 1
                sub_packet_map.update({parameter_key: [parameter_size, value]})
            self.calibration_map[key] = [sub_packet_size, sub_packet_map]
