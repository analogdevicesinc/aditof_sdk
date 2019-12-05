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
import os
import pandas as pd
from . import firmware_gen as lf
import json 
import tof_device.tof_device as tofdev #For EEPROM Read and Write
import logging
import logging.config
import numpy as np


def setup_logging():
    with open('./../logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)

'''
Predefined Hashmap key Defination
---------------------------------
'''
#Dictionary for modes
mode_dict = {'near' : 0, 'mid' : 1, 'far' : 2}

#Hashmap key for packet type
HEADER = 0
CAMERA_INTRINSIC = 1
NEAR_CAL = 2
NEAR_LF = 3
MID_CAL = 4
MID_LF = 5
FAR_CAL = 6
FAR_LF = 7


#Hashmap key for common parameters
EEPROM_VERSION = 1
CAL_SER_NUM = 2
CAL_DATE = 3
CHECKSUM = 4


#Hashmap key for Header Parameters
TOTAL_SIZE = 5
NUMBER_OF_MODES = 6
MODULE_TYPE = 11 #Value 2: BroadMarket/1: PICO/0 : ADI EVAL
AFE_TYPE = 13 #Value 0: ADDI9033 / 1:ADDI9043/ 2: ADDI9050
SENSOR_TYPE = 14 #Value 0 : Panasonic VGA / 1 : Panasonic QVGA
LASER_TYPE = 16 #Value 0 : Princeton VCSEL/ 1 : Heptagon VCSEL...

#Hashmap key for Camera Intrinsic
INTRINSIC = 5
DISTORTION_COEFFICIENTS = 6

#Hashmap for linear correct
ISATG_PROJECT_VERSION = 5
CALIBRATION_SOFTWARE_VERSION = 6
CALIBRATION_TYPE = 7 #Value 0 Sweep, 1: Rail, 2: Faceplant
CALIBRATION_MODE  = 8 #Value 0:Near 1, 1 : Mid, 2 :Far
PULSE_COUNT = 11
NO_OF_LASERS = 12
LINEAR_CORRECT_OFFSET = 22
LINEAR_CORRECT_XPWR = 23

#Hashmap for load files
ADDR_DATA_LIST = 5

#Indices for PARAM STRUCT
SIZE = 0
VALUE = 1


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
class cal_map(object):
    
    def __init__(self):
        self.calibration_map = {}
        header_packet = {
            TOTAL_SIZE : self.param_struct([8]),
            CHECKSUM : self.param_struct([8])
            }
        self.calibration_map = {
            HEADER : [self.get_packet_size(header_packet), header_packet],
            }

    #calculates size of value and returns list[size, value]
    def param_struct(self, param_value):
        size = len(param_value) * 4 # len * 4(each element is float)
        param_value = [int(size), [float(i) for i in param_value]]
        return param_value

    #calculates and returns size of packet
    def get_packet_size(self, packet):
        packet_size = 0
        for nested_key,nested_value in packet.items():
            param_size, param_value = nested_value
            packet_size = packet_size + param_size + 8 # added size 8 for key and size of each parameter
        return int(packet_size)

    #calculates and returns size of map
    def get_map_size(self):
        map_size = 0
        for key, list_params in self.calibration_map.items():
            size, nested_dict = list_params
            map_size = map_size + size
            map_size = map_size + 8 #Size of each key(4)  and packet size(4) is added(4+4=8)
        return map_size
    
    def update_packet_checksum(self, packet):
        checksum = 0
        for nested_key,nested_value in packet.items():
            param_size, param_value = nested_value
            for i in range (int(param_size/4)):
                checksum = int(checksum) ^ int(param_value[i])
        packet[CHECKSUM] = self.param_struct([checksum])
    
    def update_map_header(self):
        #Update Header Total Size
        total_size = self.get_map_size()
        self.calibration_map[HEADER][VALUE][TOTAL_SIZE] = self.param_struct([total_size])
        #Update Header Checksum
        self.update_packet_checksum(self.calibration_map[HEADER][VALUE])
        
    #Generates Default Dictionary
    def init_default_cal_map(self):
        header_packet = {
            EEPROM_VERSION : self.param_struct([0]),
            TOTAL_SIZE : self.param_struct([1000]),
            NUMBER_OF_MODES : self.param_struct([3]),
            }
        self.update_packet_checksum(header_packet)

        camera_intrinsic_packet = {
            EEPROM_VERSION : self.param_struct([0]),
            CAL_SER_NUM : self.param_struct([0]),
            CAL_DATE : self.param_struct([12042019]),
            INTRINSIC : self.param_struct([0, 0, 0, 0, 0, 0, 0, 0, 0])
        }
        self.update_packet_checksum(camera_intrinsic_packet)

        self.calibration_map = {
            HEADER : [self.get_packet_size(header_packet), header_packet],
            CAMERA_INTRINSIC : [self.get_packet_size(camera_intrinsic_packet), camera_intrinsic_packet]
            }
        #Update Header
        self.update_map_header()



    #Parses through dictionary and prints the key and value 
    def display_cal_map(self):
        #Printing just the value of Calibration Dictionary
        for key, list_params in self.calibration_map.items():
            print ("Packet Key: ", (key),end="") # print the primary key (for Packet Type)
            size, nested_dict = list_params
            print ("\tPacket Size: ", size) #print the size of pimary packet
            #print ("Packet Key: ", (key),"\tPacket Size: ", size, file=open("output.txt", "a"))
            for nested_key,nested_value in nested_dict.items():
                print("\tParam Key: ", nested_key,end="") #print the nested key (Parameter key)
                param_size, param_value = nested_value
                print("\tParam Size: ",param_size,end="") #print the size of Param
                value = []
                for i in range (int(param_size/4)):
                    value.append(param_value[i])
                print("\tParam Value: ",value) #print the value of Param
                #print("\tParam Key: ", nested_key,"\tParam Size: ",param_size,"\tParam Value: ",value, file=open("output.txt", "a")) #print the Param to file


    #Generates the binary file for writing to EEPROM
    def save_cal_map(self, filename):
        #writing float values
        f = open(filename,"wb")
        for key, list_params in self.calibration_map.items():
            f.write(struct.pack('<f', key) ) #write the primary key (for Packet Type)
            struct.pack('<f', key)
            size, nested_dict = list_params
            f.write(struct.pack('<f', size) ) #write the size of pimary packet size
            for nested_key,nested_value in nested_dict.items():
                f.write(struct.pack('<f',nested_key)) #write the nested key (Parameter key)
                param_size, param_value = nested_value
                f.write(struct.pack('<f',param_size)) #write the size of Param
                for i in range (int(param_size/4)):
                    f.write(struct.pack('<f',param_value[i])) #write the value of Param
        f.close()

    '''Reads the binary file and parses it back to map, 
    replaces the value if already exist'''
    def read_cal_map(self, filename):
        #open the file
        with open(filename,"rb") as f:
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
                while i<(sub_packet_size/4): #4:size of float
                    sub_packet_value = struct.unpack('<f', f.read(4))
                    sub_packet_value = int(sub_packet_value[0])
                    i = i + 1
                    parameter_key = sub_packet_value
                    
                    sub_packet_value = struct.unpack('<f', f.read(4))
                    sub_packet_value = int(sub_packet_value[0])
                    i = i + 1
                    parameter_size = sub_packet_value
                    
                    number_of_elements = int(parameter_size/4) #4:size of float
                    
                    value=[]
                    for j in range (number_of_elements):
                        sub_packet_value = struct.unpack('<f', f.read(4))
                        #sub_packet_value = int(sub_packet_value[0])
                        value.append(sub_packet_value[0])
                        i = i + 1
                    sub_packet_map.update({parameter_key: [parameter_size, value]})      
                self.calibration_map[key] = [sub_packet_size,sub_packet_map]
        f.close()

    #Add Load files to map, if existing map consist load files, it overwrites it, otherwise adds it
    def add_load_files_to_map(self, packet_type, lf_path):
        lf_map = {}
        lf_list =[]
        file_list = natsorted(os.listdir("./"+lf_path+"/"), alg=ns.IGNORECASE)[:13]
        #print(file_list)
        for file_name in file_list:
            if file_name.endswith(".lf"):
                addr, data, mode_locations = lf.extract_code_block("./"+lf_path+"/"+file_name)
                for i in range(len(addr)) :
                    lf_list.append(addr[i]) 
                    lf_list.append(data[i])
                #print("Parsed File", file_name,  " ", file_num, "\n", lf_list)
                #input("Press Enter to continue...")
        lf_map[ADDR_DATA_LIST] = self.param_struct(lf_list)
        #print(lf_map)
        self.update_packet_checksum(lf_map)
        self.calibration_map[packet_type] = [self.get_packet_size(lf_map), lf_map]     
        #Update Header
        self.update_map_header()
    
    def add_linear_offset_csv_to_map(self, packet_type, linear_offset_csv_file):
        linear_df = pd.read_csv(linear_offset_csv_file)
        linear_correct_offset_list  = (linear_df.to_dict(orient='list')["reg_offset_value_hex"])
        linear_correct_xpwr_list = (linear_df.to_dict(orient='list')["xcorr"][1:])
        linear_map = {}
        linear_map[LINEAR_CORRECT_OFFSET] = self.param_struct([int(i, 16) for i in linear_correct_offset_list])
        linear_map[LINEAR_CORRECT_XPWR] = self.param_struct(linear_correct_xpwr_list)
        self.calibration_map[packet_type] = [self.get_packet_size(linear_map), linear_map]
        #Update Header
        self. update_map_header()
    
    def add_json_to_map(self, packet_type, json_file):
        with open(json_file, 'r') as f:
            json_read = json.load(f)
            json_map = {}
            for key,value in json_read.items():
                for sub_key,sub_value in json_read[key].items():
                    if(type(sub_value) is list):
                        json_map[int(sub_key)] = self.param_struct(sub_value)
                    else:
                        json_map[int(sub_key)] = self.param_struct([sub_value])                       
            self.update_packet_checksum(json_map)
            self.calibration_map[packet_type] = [self.get_packet_size(json_map), json_map]
            self.update_map_header()
    
    #Function to replace calibration mode block
    def replace_eeprom_mode(self, mode, linear_cal_json_file, load_file_path):
        self.add_json_to_map((mode_dict[mode]*2+2), linear_cal_json_file)
        self.add_load_files_to_map((mode_dict[mode]*2+3), load_file_path)
    
    def write_eeprom_cal_map(self, dev):
        #print("\n\nWriting EEPROM")
        eeprom_write_bytearray = bytes()
        for key, list_params in self.calibration_map.items():
            eeprom_write_bytearray = eeprom_write_bytearray + (struct.pack('<f', key)) #write the primary key (for Packet Type)
            struct.pack('<f', key)
            size, nested_dict = list_params
            eeprom_write_bytearray = eeprom_write_bytearray + (struct.pack('<f', size)) #write the size of pimary packet size
            for nested_key,nested_value in nested_dict.items():
                eeprom_write_bytearray = eeprom_write_bytearray + (struct.pack('<f', nested_key)) #write the nested key (Parameter key)
                param_size, param_value = nested_value
                eeprom_write_bytearray = eeprom_write_bytearray + (struct.pack('<f', param_size)) #write the size of Param
                for i in range (int(param_size/4)):
                    eeprom_write_bytearray = eeprom_write_bytearray + (struct.pack('<f', param_value[i])) #write the value of Param
        eeprom_map_size = [self.get_map_size()]

        eeprom_write_list = []
        size = eeprom_write_bytearray.__len__()
        for index in range(0, size):
            eeprom_write_list.append(eeprom_write_bytearray[index])
        #print("EEPROM WRITE List\n", eeprom_write_list)
        #input("Press Enter to continue...")

        size_list = []
        size_byte = bytes()
        size_byte = struct.pack('<f', size)
        for index in range(0, 4):
            size_list.append(size_byte[index])
        dev.writeEeprom(int(0), np.array(size_list, dtype='uint8'), 4)
        dev.writeEeprom(int(4), np.array(eeprom_write_list, dtype='uint8'), eeprom_write_list.__len__())
  
    def read_eeprom_cal_map(self, dev):
        #print("Reading EEPROM")
        data_array = np.zeros(4, dtype='uint8')
        dev.readEeprom(int(0), data_array, 4)
        read_size = struct.unpack('<f', data_array)
        #print("Read Size",read_size)

        data_array = np.zeros(int(read_size[0]), dtype='uint8')
        dev.readEeprom(int(4), data_array, int(read_size[0]))
        r_b = data_array.tobytes()
        
        j = 0
        while j<r_b.__len__():
            key = r_b[j:j+4]
            j = j+4
            if not key:
                break
            key = struct.unpack('<f', key)
            key = int(key[0])
            #print("Primary Key", key)
            sub_packet_size = struct.unpack('<f', r_b[j:j+4])
            j=j+4
            sub_packet_size = int(sub_packet_size[0])
            #print("Sub Size",sub_packet_size)
            sub_packet_map = {}
            i = 0
            while i<(sub_packet_size/4): #4:size of float
                sub_packet_value = struct.unpack('<f', r_b[j:j+4])
                j=j+4
                sub_packet_value = int(sub_packet_value[0])
                i = i + 1
                parameter_key = sub_packet_value
                #print("Param Key", parameter_key)
                
                sub_packet_value = struct.unpack('<f', r_b[j:j+4])
                j=j+4
                sub_packet_value = int(sub_packet_value[0])
                i = i + 1
                parameter_size = sub_packet_value
                #print("Param Size", parameter_size)
                
                number_of_elements = int(parameter_size/4) #4:size of float
                #print("Number of elements", number_of_elements)
                value=[]
                for k in range (number_of_elements):
                    #print(r_b[j:j+4])
                    sub_packet_value = struct.unpack('<f', r_b[j:j+4])
                    j=j+4
                    #sub_packet_value = int(sub_packet_value[0])
                    value.append(sub_packet_value[0])
                    i = i + 1
                sub_packet_map.update({parameter_key: [parameter_size, value]})      
            self.calibration_map[key] = [sub_packet_size,sub_packet_map]
        
def flatten_cal_map(cal):
    lst = []
    for key, list_params in cal.items():
        size, nested_dict = list_params
        lst.append(key)
        lst.append(size)
        for nested_key,nested_value in nested_dict.items():
            param_size, param_value = nested_value
            lst.append(param_size)
            for i in range (int(param_size/4)):
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

    cal2.add_json_to_map(NEAR_CAL, "./Linear_Cal.json")
    print("\n\nCalibration map after adding Linear_Cal.json to map")
    cal2.display_cal_map()
    input("Press Enter to continue...")

    cal2.add_load_files_to_map(NEAR_LF, "../config/ADDI9043/")
    print("\n\nCalibration map after adding load files to map")
    cal2.display_cal_map()
    print("\n\nSaving to 'calibration_map.bin'")
    cal2.save_cal_map("caibration_map.bin")
    input("Press Enter to continue...")
    
    #cal2.add_linear_correct_offset(NEAR_CAL, "../saved_results/latest/linear_offset.csv")
    #cal2.display_cal_map()
    #cal2.save_cal_map()

    dev_handle = tofdev.tof_device()
    ret = dev_handle.openDevice(2,-1)
    if ret != -1:
        logger.info("Device Opened")
    else:
        logger.error("Can't open device")
        raise SystemExit
    print("\n\nWriting to EEPROM")
    cal2.write_eeprom_cal_map(dev_handle)

    print("\n\nReading from EEPROM")
    cal3 = cal_map()
    cal3.read_eeprom_cal_map(dev_handle)
    cal3.save_cal_map("eeprom_read_map.bin")
    with open ("eeprom_read_map.json", 'w') as f:
        f.write(str(cal3.calibration_map))
    f.close()
    cal3.display_cal_map()
    input("Press Enter to continue...") 

    cal3.replace_eeprom_mode('near', "./config/BM_Kit/Near/linear_cal.json", "./config/BM_Kit/Near/")
    with open ("eeprom_read_map_modified.json", 'w') as f:
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
