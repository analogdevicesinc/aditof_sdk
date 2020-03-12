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
import logging
import uuid
from datetime import datetime
import json
from distutils.dir_util import copy_tree
import os


def write_to_csv(results_path, csv_file, results_df):
    csv_path = os.path.join(results_path, csv_file)
    with open(csv_path, 'w', newline='\n') as f:
        f.write(results_df.to_csv(index=False))


def save_lf_files(latest, archive, firmware_path):

    #all firmware files
    copy_tree(firmware_path, os.path.join(latest,'lf_files'))
    copy_tree(latest, archive)


def make_results_dir(results_path, unique_id, mode):
    '''
    Make necessary results directories and store all results and configs
    '''
    logger = logging.getLogger(__name__)
    os.makedirs(results_path, exist_ok=True)
    latest = os.path.join(results_path,unique_id, mode, 'latest')
    archive = os.path.join(results_path, unique_id, mode, 'archive', datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    os.makedirs(latest, exist_ok=True)
    os.makedirs(archive, exist_ok=True)
    logger.info('latest results path %s', latest)
    logger.info('archive results path %s', archive)
    return latest, archive


def save_results(latest, archive, depth_stats_df, depth_stats_calibrated_df, linear_offset_df, cfg, firmware_path, metrics_df, cal_json):
    # results
    write_to_csv(latest, 'linear_offset.csv', linear_offset_df)
    write_to_csv(latest, 'depth_stats_pre_calibration.csv', depth_stats_df)
    write_to_csv(latest, 'depth_stats_calibrated.csv', depth_stats_calibrated_df)
    write_to_csv(latest, 'calibration_metrics.csv', metrics_df)

    # configuration
    with open(os.path.join(latest, 'input_config.json'),'w') as f:
        json.dump(cfg, f)

    with open(os.path.join(latest, 'linear_cal.json'),'w') as f:
        json.dump(cal_json, f)

    copy_tree(latest, archive)

def get_results_path(cfg_json, unique_id):
    sweep_config_dict = {}
    if(cfg_json is not None):
        with open(cfg_json) as f:
            sweep_config_dict = json.load(f)

    results_path = os.path.join(sweep_config_dict['results_path'], unique_id)
    return results_path


