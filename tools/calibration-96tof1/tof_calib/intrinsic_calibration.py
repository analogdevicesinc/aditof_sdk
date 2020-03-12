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
import time
from datetime import datetime
from natsort import natsorted, ns
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
from shutil import copyfile
import re
import re
import itertools as it
import seaborn as sns
import logging
import cv2
import json
import logging

class intrinsic_calibration:

    # Coordinates list
    rect_list = []
    
    # Click and crop variables
    refPt2 = []
    cropping = False
    imgNum = 0
    height = 480
    width = 640
    irClone = []
    irCrop = []
    
    # Calib output params
    ret = 0
    mtx = []
    dist = []
    rvecs = []
    tvecs = []
    
    # Data output options
    roi_storage = []
    
    def __init__(self):
        self.irClone = np.zeros((self.height, self.width), np.uint8)
        self.irCrop = np.zeros((self.height, self.width), np.uint8)
        self.rect_list = []
        self.refPt2 = []

    def click_and_crop(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt2.append((x, y))
            cropping = True
            
        elif event == cv2.EVENT_LBUTTONUP:
            refPt = self.refPt2
            refPt.append((x, y))
            print(refPt)
            cropping = False
            
            temp = np.zeros((self.height, self.width), np.uint8)
            temp[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]] = self.irClone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
            ret, centers = cv2.findChessboardCorners(temp, (9,6),None)
            if ret == True:
                print("Grid Found")
                self.rect_list.append((refPt[0], refPt[1]))
                # draw a rectangles around the region of interests
                self.irCrop = np.ascontiguousarray(self.irCrop, dtype=np.uint8)
                cv2.rectangle(self.irCrop, refPt[0], refPt[1], (0, 255, 0), 2)
                cv2.imshow("cropIR", self.irCrop)
                self.imgNum = self.imgNum + 1
                
            self.refPt2 = []
            
    def get_coordinates(self, irImage):
        targetPt = []
        self.irClone = irImage
        self.irCrop = irImage
        
        cv2.namedWindow("cropIR")
        cv2.setMouseCallback("cropIR", self.click_and_crop)
        while True:
            # display the image and wait for a keypress
            cv2.imshow("cropIR", self.irCrop)
            key = cv2.waitKey(1) & 0xFF

            # if the 'r' key is pressed, reset the cropping region
            if key == ord("r"):
                self.irCrop = self.irClone.copy()
                self.rect_list = []

            # if the 'c' key is pressed, break from the loop
            elif key == ord("c"):
                cv2.destroyAllWindows()
                break
                
        return self.rect_list
                
    def output_coordinates(self, path):
        dict = {}
        dict['coordinates'] = self.rect_list
        
        with open(os.path.join(path, 'intrinsic_coordinates.json'), 'w') as fp:
            json.dump(dict, fp)
            
    def load_coordinates(self, path):
        with open(os.path.join(path, 'intrinsic_coordinates.json'), 'r') as fp:
            dict = json.loads(fp.read())
            self.rect_list = dict['coordinates']
        return self.rect_list
            
    def calibrate_intrinsic(self, irImage):
        #logger = logging.getLogger(__name__)
        self.irClone = irImage
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((9*6,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
        num = 0
        
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        
        #DEBUG
        self.roi_storage = np.zeros((self.height, self.width, 5))

        for idx, i in enumerate(self.rect_list):
            roi = np.zeros((self.height, self.width), np.uint8)
            roi[i[0][1]:i[1][1], i[0][0]:i[1][0]] = self.irClone[i[0][1]:i[1][1], i[0][0]:i[1][0]]
            
            self.roi_storage[:,:,idx] = roi 
            ret, corners = cv2.findChessboardCorners(roi, (9,6),None)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(roi,corners,(9,6),(-1,-1),criteria)
                imgpoints.append(corners2)
                num = num + 1
                
        # Calibrate Camera
        if num != 0:
            self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(objpoints, imgpoints, roi.shape[::-1], None, None)
            return num, [self.ret, self.mtx, self.dist, self.rvecs, self.tvecs], self.roi_storage
        return 0, [], self.roi_storage
        #logger.debug('fx: %f, fy: %f', curr_dist, meas_depth)
        #logger.debug('cx: %f, cy: %f', curr_dist, meas_depth)
        #logger.debug('dist_matrix: %f, %f, %f, %f, %f', curr_dist, meas_depth)
        
    def output_intrinsic(self, path, sn):
        dict = {}
        mtx = np.append(self.mtx[0], np.append(self.mtx[1], self.mtx[2])).tolist()
        print(mtx)
        
        dict["Calibration_SN"] = {"2" : sn}
        dict["Calibration_Date_stamp"] = {"3" : datetime.now().strftime('%m%d%Y')}
        dict["Intrinsic"] = {"5": mtx}
        dict["Distortion_Coeff"] = {"6": self.dist[0].tolist()}
        

        os.makedirs(path, exist_ok=True)

        with open(os.path.join(path, 'camera_intrinsic.json'), 'w') as fp:
            json.dump(dict, fp)
    
    # Must be called right after calibrate_intrinsic for accurate info
    def output_intrinsic_data(self, output_path):
        
        folder_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        path = os.path.join(output_path, 'intrinsic_data', folder_name)
        
        irImage_path = os.path.join(path, 'irImage.png') 
        cv2.imwrite(irImage_path, self.irClone)
        
        print(range(self.roi_storage[-1]))
        for idx, data in enumerate(range(self.roi_storage[-1])):
            roi_path = os.path.join(path, 'roi' + str(idx) + '.png') 
            cv2.imwrite(roi_path, data)
            
        self.output_intrinsic(path)
        self.output_coordinates(path)
        
        


