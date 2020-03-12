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
import json
from natsort import natsorted, ns
from matplotlib import pyplot as plt
from . import gen_delays as gd
from . import sweep_calibration as sc
from . import intrinsic_calibration as ic
from . import write_to_lf_file as write_lf
from . import device as device
import core.frame as frame
import logging


def run_pulse_sweep(dev_handle=None, cfg=None, firmware_path=None, min_delay=None, max_delay=None):

    frame_count = cfg['frame_count']
    window = {}
    window['X'] = cfg['window_x']
    window['Y'] = cfg['window_y']
    
    pulse_count = {}
    pulse_count['init'] = cfg['pulse_count_min']
    pulse_count['max'] = cfg['pulse_count_max']
    
    raw_frame = {}
    raw_frame['height'] = cfg['raw_frame_height']
    raw_frame['width'] = cfg['raw_frame_width']
    
    frame = {}
    frame['height'] = cfg['frame_height']
    frame['width'] = cfg['frame_width']
    
    repeat_num_file = os.path.join(firmware_path,cfg['repeat_num_filename'])
    hpt_data_file = os.path.join(firmware_path,cfg['hpt_data_filename'])
    data_file = os.path.join(firmware_path,cfg['data_filename'])
    seq_file = os.path.join(firmware_path,cfg['seq_info_filename'])

    if min_delay is None:
        min_delay = cfg['min_delay']
    if max_delay is None:
        max_delay = cfg['max_delay']

    delay_dict = gd.generate_delays(hpt_data_file, data_file, min_delay, max_delay, seq_file)
    
    data_stats = sc.sx_pulse_sweep(delay_dict, raw_frame, frame, frame_count, window, dev_handle)

    return data_stats


