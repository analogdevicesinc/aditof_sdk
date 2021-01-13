/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif
#include "aditof_opencv.h"
#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"
   
#define WINDOW_NAME "ADITOF TEST"
   
using namespace aditof;

int _DisplayIR(cv::Mat *irMat, int *distanceVal);

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    bool captureStatus = false;
    bool frameReceived = false;
    bool testPassed = true;
    
    cvui::init(WINDOW_NAME);
    cv::Mat frame = cv::Mat(cv::Size(800, 480), CV_8UC3);
    frame = cv::Scalar(49, 52, 49);
    
    cv::Mat irMat;
    int distanceVal;
    const cv::_InputArray convertedMat;
    std::vector<cv::Point2f> corners;
    int flags=cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE;
    
    while(true){
    	
    	frame = cv::Scalar(49, 52, 49);
    	cvui::beginRow(frame, 50, 150);
    	cvui::beginColumn(frame, 50, 250);
    	
        if (cvui::button(frame, 730, 30, 50, 50, "X")) {
            break;
        }
        
    	if (cvui::button(frame, 30, 30, 200, 75, "Start")) {
    		captureStatus = true;
    		}
    		
    	if (cvui::button(frame, 30, 130, 200, 75, "Stop")) {
    		captureStatus = false; 
    		frameReceived = false;
    		testPassed = true;
    		}
    	
    	cvui::endColumn();
    	cvui::beginColumn(frame, 150, 250);
    	
    	if (captureStatus && !frameReceived) {
    		
    		if (_DisplayIR (&irMat, &distanceVal)) {
    			cv::resize(irMat, irMat, cv::Size(irMat.cols * 0.7,irMat.rows * 0.7), 0, 0, CV_INTER_LINEAR);
    			frameReceived = true;
    			testPassed = findChessboardCorners(irMat, cv::Size(7,7) , corners, flags);
    			captureStatus = false;
    			} 
    		else {
    			frameReceived = false;
    			captureStatus = false;
    			testPassed = false;
    			}
    		}
    		
    	if(frameReceived && testPassed) {
		cv::drawChessboardCorners(irMat, cv::Size(7,7), corners, testPassed);
    	 }
    	 
    	if(frameReceived) {
    		cvui::image(frame,260,30, irMat);
    		}
    	
    	cvui::endColumn();
    	cvui::endRow();
  
    	cvui::beginRow(frame, 50, 150);
    	cvui::beginColumn(frame, 150, 250);
    	
    	if (frameReceived && testPassed) {
    		cvui::printf(frame,30,230,1,0x00ff00,"TEST PASSED");
    	}
    	
    	if (frameReceived && !testPassed) {
    		cvui::printf(frame,30,230,1,0xff0000,"TEST FAILED:");
    		cvui::printf(frame,40,260,0.7,0xff0000,"INVALID FRAME!");
    		//cvui::printf(frame, 90, 50, "value = %d", distanceVal);
    		}
    	
    	if (!frameReceived && !testPassed) {
    		cvui::printf(frame,30,230,1,0xff0000,"TEST FAILED:");
    		cvui::printf(frame,15,260,0.7,0xff0000,"NO FRAME RECEIVED!");
    		}
    	
    	cvui::endColumn();
    	cvui::endRow();
    	
    	cvui::update();
    	cvui::imshow(WINDOW_NAME, frame);

    	if (cv::waitKey(20) == 27) {
    		break;
    	}
    }
    
    return 0;
}

int _DisplayIR(cv::Mat *irMat, int *distanceVal){
    
    Status status = Status::OK;

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraList(cameras);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found!";
        return 0;
    }

    auto camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return 0;
    }

    status = camera->setFrameType(frameTypes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return 0;
    }

    status = camera->setMode(modes[0]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }
    
    aditof::Frame frame;
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    aditof::FrameDetails frameDetails;
   
    int frameHeight = static_cast<int>(frameDetails.height);
    int frameWidth = static_cast<int>(frameDetails.width);
    int bitCount = cameraDetails.bitCount;
    
    const int smallSignalThreshold = 50;
    camera->setControl("noise_reduction_threshold",
                       std::to_string(smallSignalThreshold));
    
    for(int i = 0; i < 5; i++){          
    	/* Request frame from camera */
    	status = camera->requestFrame(&frame);
    	if (status != Status::OK) {
    	    LOG(ERROR) << "Could not request frame!";
    	    return 0;
    	}

	/* Get distance from center point */
	uint16_t *data;	
	frame.getData(aditof::FrameDataType::DEPTH, &data);

        *distanceVal = static_cast<int>(data[240 * 640 + 320]);

	/* Convert frame to IR mat */
    	status = fromFrameToIrMat(frame, *irMat);
    	if (status != Status::OK) {
    	    LOG(ERROR) << "Could not convert from frame to mat!";
    	    return 0;
    	}
	 
    	int max_value_of_IR_pixel = (1 << bitCount) - 1;
	
    	/* Distance factor IR */
    	double distance_scale_ir = 255.0 / max_value_of_IR_pixel;
	
	irMat->convertTo(*irMat, CV_8U, distance_scale_ir);
	cv::cvtColor(*irMat, *irMat, cv::COLOR_GRAY2RGB);
    }
    return 1;
}
