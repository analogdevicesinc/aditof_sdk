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
=================
REPORT GENERATION
=================

This module contains functions for generating HTML reports for calibration. 

'''

import numpy as np
import pandas as pd
import os
import json

from bokeh.layouts import grid, column, row, layout
from bokeh.models import CustomJS, Slider, ColumnDataSource, Tabs, Panel, Div
from bokeh.models.widgets import DataTable, TableColumn
from bokeh.plotting import figure, output_file, show, save
from bokeh.colors import RGB
from bokeh.io.doc import curdoc, set_curdoc

__docformat__ = 'restructuredtext'

'''
Define Colormaps
----------------
Colormap in accordance with Analog Devices branding guidelines
'''

adi_primary = [ 
                '#62B4E1',  # cyan
                '#7B40B5',  # purple
                '#4DAB6C',  # green
                '#EE7C30',  # orange
                '#D8E15F',  # yellow
                '#2966B4',  # blue
            ]

adi_light   = [ 
                '#D3E4F2',  # cyan
                '#D7CFE4',  # purple
                '#D0E1D4',  # green
                '#F9D7CD',  # orange
                '#F1F2D3',  # yellow
                '#CCD3E3',  # blue
            ]

'''
Define Constants
----------------
Define constants used for report generation
- sizeunit: Defines 1 unit for specifying sizes of each HTML canvas
- tools: tools added to each plot: 
    * pan: Mouse control for panning
    * box_zoom: Select region of interest to zoom into
    * wheel_zoom: Zoom in and out using mouse scroll
    * save: Save figure as file
    * reset: Reset all zoom and pan to default view
    * hover: Hover over data points to see their co-ordinates
'''

sizeunit = 150
tools = 'pan,box_zoom,wheel_zoom,save,reset,hover'


'''
Expected Depth Versus Measured Depth
-----------------------------------------
This function plots measured depth before and after calibration against the expected depth. 
This helps visualize nonlinearities and other peculiar behaviors.

'''
def plot_measured_depth(data):
    s1 = figure(tools=tools, title='Expected Vs Measured Depth', plot_height=6*sizeunit, plot_width=6*sizeunit)
    s1.line(x='expected_depth', y='expected_depth', source=data, line_width=0.5,  color='gray', legend='Ideal')
    s1.line(x='expected_depth', y='pre_calibration_depth', source=data, line_width=3,  color=adi_light[0], legend='Pre Calibration')
    s1.line(x='expected_depth', y='post_calibration_depth', source=data, line_width=3, color=adi_primary[0], legend='Post Calibration')
    
    s1.xaxis.axis_label = 'Expected Depth (mm)'
    s1.yaxis.axis_label = 'Measured Depth (mm)'
    s1.legend.location = 'top_left'
    
    return s1


'''
Linked Error Plots
------------------
This function plots the different error plots, all of which can be zoomed and panned in synchronization with each other. 
- Mean Depth Error (mm): Mean error in depth measurements (mm)
- Mean Depth Error (%) : Mean error in depth measurements (%)
- Depth Noise : Standard deviation of depth measurements
'''
def plot_errors(data):
    s1 = figure(tools=tools, plot_height=2*sizeunit, plot_width=2*sizeunit, title='Mean Depth Error (mm)')
    s1.line(x='expected_depth', y='pre_calibration_error_mm', source=data, line_width=2, color=adi_light[1], legend='Pre Calibration')
    s1.line(x='expected_depth', y='post_calibration_error_mm', source=data, line_width=2, color=adi_primary[1], legend='Post Calibration')
    s2 = figure(tools=tools, x_range=s1.x_range, plot_height=2*sizeunit, plot_width=2*sizeunit, title='Mean Depth Error (%)')
    s2.line(x='expected_depth', y='pre_calibration_error_percent', source=data, line_width=2, color=adi_light[2], legend='Pre Calibration')
    s2.line(x='expected_depth', y='post_calibration_error_percent', source=data, line_width=2, color=adi_primary[2], legend='Post Calibration')
    s3 = figure(tools=tools, x_range=s1.x_range, plot_height=2*sizeunit, plot_width=2*sizeunit, title='Depth Noise')
    s3.line(x='expected_depth', y='pre_calibration_noise', source=data, line_width=2, color=adi_light[3], legend='Pre Calibration')
    s3.line(x='expected_depth', y='post_calibration_noise', source=data, line_width=2, color=adi_primary[3], legend='Post Calibration')
    return [s1, s2, s3]


'''
Generate Plots
--------------
'''
def generate_plots(metrics_df):
    metrics_data = ColumnDataSource(metrics_df)
    sidebar_layout = column(plot_errors(metrics_data))
    plot_layout = row([plot_measured_depth(metrics_data), sidebar_layout])
    return plot_layout



'''
Generate Configuration Table
----------------------------
Configuration tab which contains a data table of calibration config
'''
def generate_configs_table(config_dict):
    columns = ['pname', 'pvalue']
    config_df = pd.DataFrame(columns=columns)
    config_df['pname'] = list(config_dict.keys())
    config_df['pvalue'] = list(config_dict.values())
    config_data = ColumnDataSource(config_df)

    columns = [
        TableColumn(field="pname", title="Calibration Parameter"),
        TableColumn(field="pvalue", title="Value"),
    ]
    config_table = DataTable(source=config_data, columns=columns, width=8*sizeunit, height=8*sizeunit)

    return config_table

    

'''
Generate Report
---------------
'''
def generate_report(metrics_df, config_dict, output_dir, show_report):

    # Generate HTML
    html_file_name = os.path.join(output_dir, 'calibration_report.html')
    output_file(html_file_name, mode='inline')

    header_html = '<h1>Time-of-Flight Camera Calibration Report</h1>'
    header = Div(text=header_html, sizing_mode='stretch_width')
    
    plots_layout = generate_plots(metrics_df)
    plots_tab = Panel(child=plots_layout, title='Results')

    configs_layout = generate_configs_table(config_dict)
    configs_tab = Panel(child=configs_layout, title='Configuration')
    body = Tabs(tabs=[plots_tab, configs_tab])

    html_layout = layout( [
        [header],
        [body],
    ])

    # Only show report if show_report is True
    if show_report:
        show(html_layout, title='Calibration Report')
    else:
        save(html_layout, title='Calibration Report')
    
'''
Main Function
-------------
'''
if __name__ == '__main__':
    # Get all filenames
    latest_results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'saved_results', 'latest')
    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'reportgen')
    metrics_file = os.path.join(latest_results_dir, 'calibration_metrics.csv')
    input_configs_file = os.path.join(latest_results_dir, 'input_config.json')
    

    # Load metrics dataframe
    metrics_df = pd.read_csv(metrics_file)

    config_dict = {}
    # Load calibration configurations
    with open(input_configs_file) as f:
        config_dict = json.load(f)
    generate_report(metrics_df, config_dict, latest_results_dir, show_report=True)
    


