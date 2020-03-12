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
import numpy as np
import numpy.ma as ma
import pandas as pd
import os
import time
from natsort import natsorted, ns
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
from shutil import copyfile
import re
import tof_calib.gen_delays as gd
import core.frame as frame
import re
import itertools as it
import seaborn as sns
import logging
from scipy import stats


#temp lib add
import cv2

def get_TAL_values(dev): #Ret: [TAL1_R, TAL2_R, ... , TAL6_F, TAL7_F]
    TAL_val = device.read_AFE_reg(dev, 0xc740, 14)
    for ind, x in enumerate(TAL_val):
        if (x >> 11) == 1:
            #print(ind)
            TAL_val[ind] = TAL_val[ind] | 0xF000
    return TAL_val.astype('int16')

def get_pulse_count_regs(file_path):
    regList = []
    with open(file_path) as f:
        for rep_data in f:
            hexaPattern = re.compile(r'\b[0-9a-fA-F]{4}\b')#Mataches the first 4 hex address values (16-bit) on a line
            m = re.search(hexaPattern, rep_data)
            if m:
                regList.append(int(m.group(0), 16))
    return regList


def generate_distance_list(target_distance, min_dist, max_dist, dist_step):
    '''
        Generate the list of distance steps given a target. min, max and step. 
        The distance has to pivot based on the target_distance. 
        e.g. target_distance = 10, min_dist = 3, max_dist = 15, dist_step = 3
        then the returned list is: 4, 7, 10, 13
    '''
    dist_list = np.arange(target_distance, min_dist, -dist_step)[::-1]
    dist_list = np.append(dist_list, np.arange(target_distance, max_dist, dist_step)[1::])
    return dist_list


def show_image(name, image):
    '''
     Useful for debug: stream image from camera
    '''
    plt.figure(name)
    plt.clf()
    plt.imshow(image, cmap='gray')
    plt.draw()
    plt.pause(0.00001)
    
def stream_init(image):
    ax = plt.subplot(111)
    im = ax.imshow(image)
    plt.ion()
    plt.show()
    return im
    
def stream_update(im, image):
    im.set_data(image)
    
def stream_off(im):
    nothing = 0
    
def data_stats(curr_delay, curr_dist, depth_crop, ir_crop, frame_count, window, scale_factor, sw_gain, sw_offset, cam_handle):
    # Mask Saturated/0 Pixels
    depth_masked = ma.masked_equal(depth_crop, 0)
    satCount = ma.count_masked(depth_masked)
    if satCount == frame_count * window['X'] * window['Y']:
        meas_depth = 0
    else:
        meas_depth = np.mean(depth_masked)
    
    meas_depth_14b = meas_depth*scale_factor
    expected_depth_14b = (curr_dist*scale_factor)
    correction_offset_14b = (((curr_dist - sw_offset) / sw_gain)*4) - meas_depth_14b
    meas_depth_adj = (np.median(depth_crop) * sw_gain) + sw_offset
    meas_ir = np.median(ir_crop)
    
    #Get TAL Values
    TAL_values = get_TAL_values(cam_handle)
    
    depth_std = np.std(depth_crop) * sw_gain
    depth_noise = 100 * depth_std / meas_depth_adj

    ir_std = np.std(ir_crop)
    ir_noise = 100 * ir_std / meas_ir

    error = meas_depth_adj - curr_dist
    
    return [curr_delay, curr_dist, meas_depth, meas_depth_adj, error, meas_ir, \
    satCount, depth_std, depth_noise, ir_std, ir_noise, \
    TAL_values[0], TAL_values[7], TAL_values[1], TAL_values[8], TAL_values[2], TAL_values[9], TAL_values[3], TAL_values[10], TAL_values[4], TAL_values[11], \
    expected_depth_14b, meas_depth_14b, correction_offset_14b]
    

def pulse_sweep(delay_dict, min_dist, max_dist, dist_step, target_distance, dist_interval, raw_frame_dict, frame_dict, frame_count, window, scale_factor, sw_gain, sw_offset, cam_handle):

    logger = logging.getLogger(__name__)
    logger.info('Running Delay Sweep')
    min_delay = min(delay_dict.keys())
    max_delay = max(delay_dict.keys())
    delay_list = np.arange(min_delay, max_delay)#[::-1]
    dist_list = generate_distance_list(target_distance, min_dist, max_dist, dist_step)
    depth_crop = np.empty([len(dist_list), frame_count, window['X'], window['Y']])
    ir_crop = np.empty([len(dist_list), frame_count, window['X'], window['Y']])
    results = []
    for step, curr_dist in enumerate(dist_list):
        delay_list_ind = np.searchsorted(delay_list, round((target_distance-curr_dist)/dist_interval))
        curr_delay = delay_list[delay_list_ind]
        logger.debug('curr_delay: %d, curr_dist: %d', curr_delay, curr_dist)
        addr_list = [int(x,16) for x in delay_dict[curr_delay].keys()]
        value_list = [x for x in delay_dict[curr_delay].values()]
        device.write_AFE_reg(cam_handle, addr_list, value_list)
        #for addr, value in zip(addr_list, value_list):
        #    print('a: ' + hex(addr) + ' v: ' + hex(value))
        #dummy reads
        frame.dummy_read(cam_handle)
        for frame_ind in np.arange(0, frame_count):
            depth_image, ir_image = frame.get_depth_ir_images_df(cam_handle, raw_frame_dict['width'], raw_frame_dict['height'])
            depth_crop[step][frame_ind] = frame.crop_center(depth_image, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
            ir_crop[step][frame_ind] = frame.crop_center(ir_image, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
        
        #Process Data
        data = data_stats(curr_delay, curr_dist, depth_crop[step], ir_crop[step], frame_count, window, scale_factor, sw_gain, sw_offset, cam_handle)
        results.append(data)
        
        logger.debug('expected_depth: %d, measured_depth: %d', data[1], data[3])
    
    # Reset Timing Registers
    addr_list = [int(x,16) for x in delay_dict[0].keys()]
    value_list = [x for x in delay_dict[0].values()]
    device.write_AFE_reg(cam_handle, addr_list, value_list)
    
    dfResults = pd.DataFrame(results, columns = ['delay','expected_depth_12b', 'meas_depth_12b', 'meas_depth_12b_adj', 'error', 'meas_ir_12b', \
                                                'saturation_count', 'depth_std', 'depth_noise', 'ir_std', 'ir_noise', \
                                                'TAL1_R','TAL1_F','TAL2_R','TAL2_F','TAL3_R','TAL3_F','TAL4_R','TAL4_F','TAL5_R','TAL5_F', \
                                                'expected_depth_14b', 'meas_depth_14b', 'correction_offset_14b'])
    return dfResults


def sx_pulse_sweep(delay_dict, raw_frame_dict, frame_dict, frame_count, window, dev):

    logger = logging.getLogger(__name__)
    logger.info('Running Delay Sweep')
    min_delay = min(delay_dict.keys())
    max_delay = max(delay_dict.keys())
    delay_list = np.arange(max_delay, min_delay, -4)
    data1Crop = np.empty([len(delay_list), frame_count, window['X'], window['Y']])
    data2Crop = np.empty([len(delay_list), frame_count, window['X'], window['Y']])
    results = []

    for step, delay_list_ind in enumerate(delay_list):
        logger.debug('curr_delay: %d', delay_list_ind)
        addr_list = [int(x,16) for x in delay_dict[delay_list_ind].keys()]
        value_list = [x for x in delay_dict[delay_list_ind].values()]
        write_to_AFE(dev, addr_list, value_list)

        #dummy reads
        frame.dummy_read(dev)
        for frame_ind in np.arange(0, frame_count):
            data1, data2 = frame.get_depth_ir_images_df(dev, raw_frame_dict['width'], raw_frame_dict['height'])
            data1Crop[step][frame_ind] = frame.crop_center(data1, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
            data2Crop[step][frame_ind] = frame.crop_center(data2, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
        data1Avg = np.median(data1Crop[step])
        data2Avg = np.median(data2Crop[step])
        results.append([-delay_list_ind, data1Avg, data2Avg])
    
    # Reset Timing Registers
    addr_list = [int(x,16) for x in delay_dict[0].keys()]
    value_list = [x for x in delay_dict[0].values()]
    write_to_AFE(dev, addr_list, value_list)
    
    return results


def calc_xpower(xcorr):
    xpow = []
    cum_sum = 0
    for ind, x in enumerate(xcorr):
        cum_sum += pow(2,x)
        xpow.append(cum_sum)
    return xpow

def calc_non_linear_offset(xcorr, depth_stats_df):
    '''
    Calculate the non-linear offset correction values
    Args:
        xcorr (list of int): The X correlation values. This is fixed for a given sensor/AFE
        depth_stats_df (pandas dataframe): This is the dataframe and statistics gathered as part of the sweep

    Returns:
        linear_offset_df: The dataframe with the linear offset correction values
    '''
    logger = logging.getLogger(__name__)
    logger.info('Calculating Non-Linear offsets')
    xpower = calc_xpower(xcorr)
    linear_offset_df = pd.DataFrame(data={'xcorr': xcorr, 'depth_in': xpower}) 
    # https://docs.scipy.org/doc/numpy/reference/generated/numpy.searchsorted.html
    depth_index = np.searchsorted(depth_stats_df['meas_depth_14b'], linear_offset_df['depth_in'])
    corrected_offset = []
    # iterate over pair of consecutive indices
    # https://stackoverflow.com/questions/21303224/iterate-over-all-pairs-of-consecutive-items-in-a-list
    for count, index in enumerate(depth_index):
        first = index-1
        second = index

        # corner cases
        if index == 0:
            first = index
            second = index + 1
        # index greater than the length of measured data
        if index >= len(depth_stats_df['meas_depth_14b'])-1:
            first = index - 1
            second = first

        x = [depth_stats_df['meas_depth_14b'][first], depth_stats_df['meas_depth_14b'][second]]
        y = [depth_stats_df['correction_offset_14b'][first], depth_stats_df['correction_offset_14b'][second]]
        corrected_offset.append(np.interp(linear_offset_df['depth_in'][count], x, y))
    linear_offset_df['corrected_offset'] = corrected_offset
    return linear_offset_df
    
def calc_non_linear_offset2(xcorr, depth_stats_df, sw_gain, sw_offset):
    logger = logging.getLogger(__name__)
    logger.info('Calculating Non-Linear offsets')
    
    #Fit curve
    depth_stats_df['shifted_expected_depth_14b'] = (depth_stats_df['expected_depth_14b']) - (sw_offset*4)
    #print(depth_stats_df['shifted_expected_depth_14b'])
    #print(type(depth_stats_df['meas_depth_14b'][2]))
    
    slope, intercept, r_value, p_value, std_err = stats.linregress(depth_stats_df['shifted_expected_depth_14b'].copy(),depth_stats_df['meas_depth_14b'].copy())
    depth_stats_df['fitted_curve'] = slope*depth_stats_df['shifted_expected_depth_14b']+intercept
    depth_stats_df['correction_offset_14b'] = depth_stats_df['fitted_curve'] - depth_stats_df['meas_depth_14b']
    
    gain = (.25*(1/sw_gain))/slope
    offset = -intercept
    
    xpower = calc_xpower(xcorr)
    linear_offset_df = pd.DataFrame(data={'xcorr': xcorr, 'depth_in': xpower}) 
    # https://docs.scipy.org/doc/numpy/reference/generated/numpy.searchsorted.html
    depth_index = np.searchsorted(depth_stats_df['meas_depth_14b'], linear_offset_df['depth_in'])
    corrected_offset = []
    # iterate over pair of consecutive indices
    # https://stackoverflow.com/questions/21303224/iterate-over-all-pairs-of-consecutive-items-in-a-list
    for count, index in enumerate(depth_index):
        first = index-1
        second = index

        # corner cases
        if index == 0:
            first = index
            second = index + 1
        # index greater than the length of measured data
        if index >= len(depth_stats_df['meas_depth_14b'])-1:
            first = index - 1
            second = first

        x = [depth_stats_df['meas_depth_14b'][first], depth_stats_df['meas_depth_14b'][second]]
        y = [depth_stats_df['correction_offset_14b'][first], depth_stats_df['correction_offset_14b'][second]]
        corrected_offset.append(np.interp(linear_offset_df['depth_in'][count], x, y))
    linear_offset_df['corrected_offset'] = corrected_offset
    linear_offset_df['gain'] = gain
    linear_offset_df['offset'] = offset
    
    #print(gain)
    #print(offset)
    
    return linear_offset_df, depth_stats_df
    
def rail_sweep (min_dist, max_dist, dist_step, raw_frame_dict, frame_dict, frame_count, window, scale_factor, sw_gain, sw_offset, rail, cam_handle):
    logger = logging.getLogger(__name__)
    logger.info('Running Rail Sweep')
    
    depthCrop = np.empty([frame_count, window['X'], window['Y']])
    results = []
    
    img = None
    rail.moveRailMM(int(min_dist))
    time.sleep(1) #inital wait
    
    for i, curr_loc in enumerate(range(int(min_dist), int(max_dist+1), int(dist_step))):
        rail.moveRailMM(curr_loc)
        #Wait time for target to stop shaking
        time.sleep(5)
        frame.dummy_read(cam_handle)
        
        for frame_ind in np.arange(0, frame_count):
            depthImage = frame.get_depth_image_df(cam_handle, raw_frame_dict['width'], raw_frame_dict['height'])
            depthCrop[frame_ind] = frame.crop_center(depthImage, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
        
        depthMasked = ma.masked_equal(depthCrop,0)
        satCount = ma.count_masked(depthMasked)
        meas_depth = np.median(depthCrop)

        meas_depth_14b = meas_depth*scale_factor
        expected_depth_14b = curr_loc * scale_factor
        correction_offset_14b = (((curr_loc - sw_offset) / sw_gain)*4) - meas_depth_14b
        meas_depth = (np.median(depthCrop) * sw_gain) + sw_offset
        
        results.append([0, curr_loc, expected_depth_14b, meas_depth, meas_depth_14b, correction_offset_14b, satCount])
        logger.debug('expected_depth: %d, measured_depth: %d', curr_loc, meas_depth)
    dfResults = pd.DataFrame(results, columns = ['delay', 'expected_depth_12b','expected_depth_14b', 'meas_depth_12b', 'meas_depth_14b', 'correction_offset_14b', 'saturation_count'])
    
    return dfResults

def rail_stats (min_dist, max_dist, dist_step, raw_frame_dict, frame_dict, frame_count, window, scale_factor, sw_gain, sw_offset, rail, cam_handle):
    logger = logging.getLogger(__name__)
    logger.info('Running Rail Sweep')
    
    depth_crop = np.empty([frame_count, window['X'], window['Y']])
    ir_crop = np.empty([frame_count, window['X'], window['Y']])
    results = []
    
    img = None
    rail.moveRailMM(int(min_dist))
    
    for i, curr_loc in enumerate(range(int(min_dist), int(max_dist+1), int(dist_step))):
        rail.moveRailMM(curr_loc)
        frame.dummy_read(cam_handle)
        
        for frame_ind in np.arange(0, frame_count):
            depthImage, irImage = frame.get_depth_ir_images_df(cam_handle, raw_frame_dict['width'], raw_frame_dict['height'])
            depth_crop[frame_ind] = frame.crop_center(depthImage, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
            ir_crop[frame_ind] = frame.crop_center(irImage, frame_dict['width'], frame_dict['height'], window['X'], window['Y'])
        
        data = data_stats(i, curr_loc, depth_crop, ir_crop, frame_count, window, scale_factor, sw_gain, sw_offset, cam_handle)
        results.append(data)
        
        logger.debug('expected_depth: %d, measured_depth: %d', curr_loc, data[3])
        
    dfResults = pd.DataFrame(results, columns = ['delay','expected_depth_12b', 'meas_depth_12b', 'meas_depth_12b_adj', 'error', 'meas_ir_12b', \
                                            'saturation_count', 'depth_std', 'depth_noise', 'ir_std', 'ir_noise', \
                                            'TAL1_R','TAL1_F','TAL2_R','TAL2_F','TAL3_R','TAL3_F','TAL4_R','TAL4_F','TAL5_R','TAL5_F', \
                                            'expected_depth_14b', 'meas_depth_14b', 'correction_offset_14b'])
    
    return dfResults

