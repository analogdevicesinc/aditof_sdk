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
from tempfile import mkstemp
import os
import re
import numpy as np
import logging

# def float_bin(number, places = 3): 
    # whole, dec = str(number).split(".") 
    # whole = int(whole) 
    # dec = int (dec) 
    # decimal = bin(whole).lstrip("0b")
    # res = ''
    # for x in range(places): 
        # whole, dec = str((decimal_converter(dec)) * 2).split(".") 
        # dec = int(dec) 
        # res += whole 
  
    # if decimal == '':
        # decimal = '0'
    # if res == '':
        # res == '0'
    # return decimal, res 
    
def float_bin(number , places=3):
    t = number
    integral = 0
    while t > 1:
        t = t - 1
        integral = integral + 1
    
    frac_str = ''
    for x in range(places):
        t = t * 2
        if t >= 1:
            frac_str = frac_str + '1'
            t=t-1
        else:
            frac_str = frac_str + '0'
            
    return str(integral), frac_str
  
def decimal_converter(num):
    if num == 0:
        return 0.0
    while num > 1: 
        num /= 10
    return num 
    
#converts to u2.12
def to_fixed_point(n):
    number = float_bin(n, places = 12)
    hex_num = tohex((int(number[0],10) <<12) + int(number[1],2), 16, '04x')    
    return hex_num

def tohex(val, nbits, format_string):
    return format((int(val) + (1 << nbits)) % (1 << nbits),format_string)


def replace_value_lf_files(reg_values, pattern, input_file, output_file):
    ind = 0
    with open(input_file) as f_in:
        with open(output_file, 'w') as f_out:
            pattern = re.compile(pattern)
            for line in f_in:        
                m = re.search(pattern, line)
                if m: 
                   addr, value, marker, comment = m.group(0).split()
                   #print('addr: ' + str(addr) + ' value:' + str(value))
                   f_out.write('    '.join([addr, reg_values[ind], marker, comment, '\n']))
                   #print('    '.join([addr, reg_values[ind], marker, comment, '\n']))
                   ind += 1
                else:
                   f_out.write(line)


def write_linear_offset_to_lf(firmware_path, results_path, cfg, linear_offset_df):
    logger = logging.getLogger(__name__)
    logger.info('Writing generated non-linear offsets to lf file %s', cfg['non_linear_off_lf_file'])
    input_file = os.path.join(firmware_path, cfg['non_linear_off_lf_file'])
    output_file = os.path.join(results_path, cfg['non_linear_off_lf_file'])
    reg_values = cfg['xcorr'] 
    hex_reg_values = [tohex(value, 4, '01x') for value in reg_values]

    concat_4_reg_values = ['0000']
    for ind in np.arange(0, len(hex_reg_values), 4):
        concat_4_reg_values.append(''.join(hex_reg_values[ind+4:ind:-1]))

    fd, file_path = mkstemp()
    replace_value_lf_files(concat_4_reg_values, r'.*LNR_X.*', input_file, file_path)

    linear_offset_df['reg_offset_value_hex'] = linear_offset_df['corrected_offset'].apply(lambda x: tohex(x, 14, '04x'))

    replace_value_lf_files(linear_offset_df['reg_offset_value_hex'] , r'.*LNR_OFST.*',file_path, output_file)

def write_linear_offset_to_lf2(firmware_path, results_path, cfg, linear_offset_df):
    logger = logging.getLogger(__name__)
    logger.info('Writing generated non-linear offsets to lf file %s', cfg['non_linear_off_lf_file'])
    input_file = os.path.join(firmware_path, cfg['non_linear_off_lf_file'])
    output_file = os.path.join(results_path, cfg['non_linear_off_lf_file'])
    reg_values = cfg['xcorr'] 
    hex_reg_values = [tohex(value, 4, '01x') for value in reg_values]

    concat_4_reg_values = ['0000']
    for ind in np.arange(0, len(hex_reg_values), 4):
        concat_4_reg_values.append(''.join(hex_reg_values[ind+4:ind:-1]))

    fd, file_path = mkstemp()
    fd1, file_path1 = mkstemp()
    fd2, file_path2 = mkstemp()
    
    replace_value_lf_files(concat_4_reg_values, r'.*LNR_X.*', input_file, file_path)

    linear_offset_df['reg_offset_value_hex'] = linear_offset_df['corrected_offset'].apply(lambda x: tohex(x, 14, '04x'))

    replace_value_lf_files(linear_offset_df['reg_offset_value_hex'] , r'.*LNR_OFST.*',file_path, file_path1)
    
    #print(linear_offset_df['gain'][0])
    gain_h = to_fixed_point(linear_offset_df['gain'][0])
    offset_h = tohex(int(linear_offset_df['offset'][0]), 15, '04x')
    
    #print(gain_h)
    #print(offset_h)
    
    replace_value_lf_files([gain_h], r'.*GAIN.*',file_path1, file_path2)
    replace_value_lf_files([offset_h], r'.*OFFSET.*',file_path2, output_file)

