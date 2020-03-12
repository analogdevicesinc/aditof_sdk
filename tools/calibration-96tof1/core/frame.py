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
import tof_calib.device as device

# def get_depth_image_df(dev, width, height):
    # image = dev.getImage()
    # image = np.resize(image, (height, width))
    # depthImage = pd.DataFrame(image[0::2, :])
    # return depthImage
    
def get_depth_image_df(cam_handle, width, height):
    image = device.get_depth_image(cam_handle)
    depth_image = pd.DataFrame(image)
    return depth_image

def get_ir_image_df(cam_handle, width, height):
    image = device.get_ir_image(cam_handle)
    ir_image = pd.DataFrame(image)
    return ir_image
    
# def get_ir_image_df(dev, width, height):
    # image = dev.getImage()
    # image = np.resize(image, (height, width))
    # irImage = pd.DataFrame(image[1::2, :])
    # return irImage

# def get_depth_ir_images_df(dev, width, height):
    # image = dev.getImage()
    # image = np.resize(image, (height, width))
    # depthImage = pd.DataFrame(image[0::2, :])
    # irImage = pd.DataFrame(image[1::2, :])
    # return depthImage, irImage
    
def get_depth_ir_images_df(cam_handle, width, height):
    depth_image_n, ir_image_n = device.get_depth_ir_image(cam_handle)
    depth_image = pd.DataFrame(depth_image_n)
    ir_image = pd.DataFrame(ir_image_n)
    return depth_image, ir_image

def crop_center(image, frame_width, frame_height, x, y):
    return image.iloc[int((frame_height-y)/2):int((frame_height+y)/2), int((frame_width-x)/2):int((frame_width+x)/2)]

def get_ir_image(dev):

    image = dev.getImage()
    image = np.resize(image, (960,640))
    irImage = image[1::2, :]
    imax = np.amax(irImage)
    irImage = np.uint8(255*(irImage/imax))
    irImage.resize((480,640))
    return irImage


def dummy_read(cam_handle):
    device.get_ir_image(cam_handle)
    device.get_ir_image(cam_handle)
 
