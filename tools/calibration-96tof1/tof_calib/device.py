#import tof_device.tof_device as tofdev
import aditofpython as tof
import cal_eeprom.firmware_gen as fgen
from natsort import natsorted, ns
import logging
import os
import numpy as np
import struct

def mode_stop(dev):
    dev.writeAFEReg(0x4001, 0x0006)
    dev.writeAFEReg(0x7c22, 0x0004)


def mode_start(dev):
    dev.writeAFEReg(0x4001, 0x0007)
    dev.writeAFEReg(0x7c22, 0x0004)
    
def write_AFE_reg(cam_handle, addr, data):
    logger = logging.getLogger(__name__)
    if len(addr) != len(data):
        logger.error('Address and Data arrays not same length. Cannot program registers')
        return -1
    
    address_array = np.array(addr, dtype='uint16')
    data_array = np.array(data, dtype='uint16')
    
    dev = cam_handle.getDevice()
    
    status = dev.writeAfeRegisters(address_array, data_array, len(data_array))
    #logger.debug(status)
    
def read_AFE_reg(cam_handle, addr, length):
    logger = logging.getLogger(__name__)
    if length <= 0:
        logger.error('Invalid length')
        return -1
    
    addr_list = []
    for i in range(length):
        addr_list.append(addr+i)
    
    address_array = np.array(addr_list, dtype='uint16')
    data_array = np.zeros(length, dtype='uint16')
    
    dev = cam_handle.getDevice()
    status = dev.readAfeRegisters(address_array, data_array, length)
    #logger.debug(status)
    
    return data_array
    
def extract_code_block(lf_filename):
   f = open(lf_filename)
   address = []
   data = []
   mode_locations={}
   lineNum=0
   for line in f.readlines():
       # Clean up line to remove comments and extract hexa codes
       removedChars = ' \t\n\r'
       for char in removedChars:
         line = line.replace(char,"")

       if (len(line)>=8):
         try:
           addr=int(line[0:4],16)
           dat=int(line[4:8],16)
           address.append(addr)
           data.append(dat)
           # Check if there is a //.. or /* ... */ describing pattern, mode, field and sequence numbers
           if '//' in line:
             name=line.split('//',1)[1]
             mode_locations[name] = lineNum
           if '/*' in line:
             name=line[line.index('/*')+2:line.index('*/')]
             mode_locations[name] = lineNum
           lineNum+=1

         except ValueError:
           continue
   f.close()
   return address, data, mode_locations


def generate_bin(addr_data_tuple, bin_file_name):
    with open(bin_file_name, 'wb') as f:
        for addr,data in addr_data_tuple:
           # convert to binary data - little endian
           bin_addr = struct.pack('<H', addr)
           bin_data = struct.pack('<H', data)
           f.write(bin_addr)
           f.write(bin_data)

def open_device2(sys_handle):
    logger = logging.getLogger(__name__)
    
    #Find cameras
    cameras = []
    status = sys_handle.getCameraList(cameras)
    
    logger.info("Get Cameras: " + str(status))
    if len(cameras) > 0:
        logger.info("Cameras Found: " + str(len(cameras)))
        print('found: ' + str(len(cameras)))
    else:
        logger.error("No Cameras Found")
        raise SystemExit
    
    cam_handle = cameras[0]
    cam_handle.initialize()
    
    camDetails = tof.CameraDetails()
    status = cam_handle.getDetails(camDetails)
    logger.info("system.getDetails()" + str(status))
    logger.info("cam_handle details:" + "id:" + str(camDetails.cameraId) + "connection:" + str(camDetails.connection))
    print(camDetails.cameraId)
    
    return cam_handle
 
 
def open_device():
    logger = logging.getLogger(__name__)
    dev_handle = tofdev.tof_device()
    ret = dev_handle.openDevice(2,-1)
    if ret == 0:
        logger.info("Device Opened")
    else:
        logger.error("Can't open device")
        raise SystemExit
    return dev_handle
 
def program_firmware2(cam_handle, firmware_path):
    logger = logging.getLogger(__name__)
    logger.info("Firmware Path %s", firmware_path)
    file_list = natsorted(os.listdir("./"+firmware_path+"/"), alg=ns.IGNORECASE)
    lf_file_list = []
    for file_name in file_list:
        if file_name.endswith(".lf"):
            lf_file_list.append(file_name)

    if len(lf_file_list) > 13:
        logger.error("More than 13 Lf files found at %s", firmware_path)
        print(lf_file_list)
        raise SystemExit
    
    address = []
    data = []
    for file_name in lf_file_list:
        a, d, mode = extract_code_block(os.path.join(firmware_path, file_name))
        address = address + a
        data = data + d
        
    addr_data_zip = zip(address, data)
    generate_bin(addr_data_zip, 'afe_firmware.bin')
    
    logger.info('Frame type set to: depth_ir')
    
    types = []
    cam_handle.getAvailableFrameTypes(types)
    logger.info(str(types))
    cam_handle.setFrameType(types[0])
    
    modes = []
    cam_handle.getAvailableModes(modes)
    logger.info('Mode set to calibration, programming device')
    logger.info(str(modes))
    cam_handle.setMode(modes[3], 'afe_firmware.bin')
    
def get_depth_image(cam_handle):
    logger = logging.getLogger(__name__)
    
    frame = tof.Frame()
    status = cam_handle.requestFrame(frame)
    #logger.debug(status)
    
    depth_image = np.array(frame.getData(tof.FrameDataType.Depth), copy=True)
    return depth_image
    
def get_ir_image(cam_handle):
    logger = logging.getLogger(__name__)
    
    frame = tof.Frame()
    status = cam_handle.requestFrame(frame)
    #logger.debug(status)
    
    ir_image = np.array(frame.getData(tof.FrameDataType.IR), copy=True)
    return ir_image
    
def get_depth_ir_image(cam_handle):
    logger = logging.getLogger(__name__)
    
    frame = tof.Frame()
    status = cam_handle.requestFrame(frame)
    #logger.debug(status)
    
    depth_image = np.array(frame.getData(tof.FrameDataType.Depth), copy=True)
    ir_image = np.array(frame.getData(tof.FrameDataType.IR), copy=True)

    return depth_image, ir_image
    

def program_firmware(dev_handle, firmware_path):
    logger = logging.getLogger(__name__)
    logger.info("Firmware Path %s", firmware_path)
    file_list = natsorted(os.listdir("./"+firmware_path+"/"), alg=ns.IGNORECASE)
    lf_file_list = []
    for file_name in file_list:
        if file_name.endswith(".lf"):
            lf_file_list.append(file_name)

    if len(lf_file_list) > 13:
        logger.error("More than 13 Lf files found at %s", firmware_path)
        print(lf_file_list)
        raise SystemExit

    for file_name in lf_file_list:
        #print('Parsing:' + file_name)
        dev_handle.parseCode("./"+firmware_path+"/"+file_name)

    mode_stop(dev_handle)
    dev_handle.program()
    logger.info("Device Programmed")
    # TURN on ISA Mode
    dev_handle.writeAFEReg(int('0xC0A9', 16), 2)
    mode_start(dev_handle)


