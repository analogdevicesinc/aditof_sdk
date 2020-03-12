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
Metrics Calculation
===================

This module contains functions for calculating statistical metrics for a depth sensor. 

'''

import numpy as np
import pandas as pd
    
__docformat__ = 'restructuredtext'
filepath = __file__

'''
CALCULATE DEPTH ERROR
---------------------
Percentage mean error observed in the depth measurements.

:param measured_depth : Pandas series. Each item is a list of measurements for a specific calibration point
:param expected_depth : Pandas series containing expected depth value at the calibration points
:returns: Two pandas series containing absolute and percentage mean error at each calibration point
'''

def calculate_depth_error(measured_depth, expected_depth):
    mean_measured_depth     =   measured_depth.apply(np.mean)
    absolute_depth_error    =   np.absolute((mean_measured_depth - expected_depth))
    percent_depth_error     =   np.divide(absolute_depth_error*100, expected_depth)
    return absolute_depth_error, percent_depth_error


'''
CALCULATE DEPTH NOISE
-------------------------
Standard deviation of the noise observed in depth measurements

:param measured_depth : Pandas series. Each item is a list of measurements for a specific calibration point
:returns: A pandas series containing noise of depth measurements at each calibration point
'''
def calculate_depth_noise(measured_depth):
    depth_noise = measured_depth.apply(np.std)
    return depth_noise


'''
CALCULATE PRE AND POST CALIBRATION METRICS
------------------------------------------
This function calculates the metrics before and after calibration

:param depth_stats_pre_calibration : A dataframe containing pre-calibration depth statistics
:param depth_stats_post_calibration : A dataframe containing post-calibration depth statistics
:returns: A dataframe containing columns ['expected_depth', 'pre_calibration_error_mm', 
    'pre_calibration_error_percent', 'pre_calibration_noise', 'post_calibration_error_mm' 
    'post_calibration_error_abs','post_calibration_noise']
'''
def calculate_metrics(depth_stats_pre_calibration, depth_stats_post_calibration):

    delay               =   depth_stats_pre_calibration['delay']
    expected_depth      =   depth_stats_pre_calibration['expected_depth_12b']
    measured_depth      =   depth_stats_pre_calibration['meas_depth_12b']
    expected_depth_post =   depth_stats_post_calibration['expected_depth_12b']
    measured_depth_post =   depth_stats_post_calibration['meas_depth_12b']

    pre_calibration_error_mm, pre_calibration_error_percent    =   calculate_depth_error(measured_depth, expected_depth)
    pre_calibration_noise   =   calculate_depth_noise(measured_depth)
    post_calibration_error_mm, post_calibration_error_percent   =   calculate_depth_error(measured_depth_post, expected_depth_post)
    post_calibration_noise  =   calculate_depth_noise(measured_depth_post)

    metrics_df = pd.DataFrame(columns= ['delay', \
                                        'expected_depth', \
                                        'pre_calibration_error_mm', \
                                        'pre_calibration_error_percent', \
                                        'pre_calibration_noise', \
                                        'post_calibration_error_mm', \
                                        'post_calibration_error_percent', \
                                        'post_calibration_noise'])

    metrics_df['delay']                             =   delay
    metrics_df['expected_depth']                    =   expected_depth
    metrics_df['pre_calibration_error_mm']          =   pre_calibration_error_mm
    metrics_df['pre_calibration_error_percent']     =   pre_calibration_error_percent
    metrics_df['pre_calibration_noise']             =   pre_calibration_noise
    metrics_df['post_calibration_error_mm']         =   post_calibration_error_mm
    metrics_df['post_calibration_error_percent']    =   post_calibration_error_percent
    metrics_df['post_calibration_noise']            =   post_calibration_noise

    return metrics_df


'''
MAIN
____
Main function invoked from the command line
Uses the latest post calibration depth measurement statistics to calculate metrics
:returns:

'''
if __name__ == "__main__":
    import csv
    import os
    latest_results_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'saved_results', 'latest')
    depth_stats_pre_calibration_file = os.path.join(latest_results_path, "depth_stats_pre_calibration.csv")
    depth_stats_post_calibration_file = os.path.join(latest_results_path, "depth_stats_calibrated.csv")
    metrics_file = os.path.join(latest_results_path, "calibration_metrics.csv")
    
    depth_stats_pre_calibration = pd.read_csv(depth_stats_post_calibration_file)
    depth_stats_post_calibration = pd.read_csv(depth_stats_post_calibration_file)

    metrics_df = calculate_metrics(depth_stats_pre_calibration, depth_stats_post_calibration)

    with open(metrics_file, 'w+', newline='\n') as f:
        f.write(metrics_df.to_csv(index=False))
    
    min_dist_row = 0
    target_row = pd.Index(depth_stats_pre_calibration['delay']).get_loc(0)
    max_dist_row = metrics_df.shape[0] - 1 
    print("Calculating metrics for the latest run")
    print("Metrics at min_dist, target_dist and max_dist:")
    print(metrics_df.iloc[[min_dist_row, target_row, max_dist_row], :])
    
    
