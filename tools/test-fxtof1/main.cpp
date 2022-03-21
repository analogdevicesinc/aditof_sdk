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
#include <aditof/storage_interface.h>
#include <aditof/system.h>
#include <aditof/temperature_sensor_interface.h>
#ifdef DATA_HANDLING
#include <filesystem>
#endif
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <iostream>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <fstream>

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

#define WINDOW_NAME "ADITOF-TEST"

using namespace aditof;

int _DisplayIR(cv::Mat *irMat, int *measuredDistance, int targetDistance,
               float *precision, float *temperature);

#ifdef DATA_HANDLING
void saveData(cv::Mat irMat, cv::Mat depthMath, std::string eepromID);
void moveData(std::string eepromID);
#endif

int main(int argc, char *argv[]) {

#ifndef JS_BINDINGS 
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
#endif

    bool captureStatus = false;
    bool frameReceived = false;
    bool testPassed = true;

    bool fullScreen = false;

    cvui::init(WINDOW_NAME);
    cv::Mat frame = cv::Mat(cv::Size(800, 480), CV_8UC3);
    frame = cv::Scalar(49, 52, 49);

    cv::Mat irMat;

    const cv::_InputArray convertedMat;
    std::vector<cv::Point2f> corners;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE;

    int measuredDistance = 0;
    int targetDistance = 500;
    float precision;
    float temperature = 0;

    while (true) {

        frame = cv::Scalar(49, 52, 49);
        cvui::beginRow(frame, 50, 150);
        cvui::beginColumn(frame, 50, 250);

        if (cvui::button(frame, 730, 30, 50, 50, "X")) {
            break;
        }

        if (cvui::button(frame, 710, 100, 70, 200, "PowerOff")) {
            system("shutdown -P now");
        }

        if (cvui::button(frame, 30, 30, 200, 75, "Start")) {
            captureStatus = true;
        }

        if (cvui::button(frame, 30, 130, 200, 75, "Stop")) {
            captureStatus = false;
            frameReceived = false;
            testPassed = true;
            measuredDistance = 0;
        }

        cvui::endColumn();
        cvui::beginColumn(frame, 150, 250);

        if (captureStatus && !frameReceived) {

            if (_DisplayIR(&irMat, &measuredDistance, targetDistance,
                           &precision, &temperature)) {
                cv::resize(irMat, irMat,
                           cv::Size(irMat.cols * 0.65, irMat.rows * 0.65), 0, 0,
                           CV_INTER_LINEAR);
                frameReceived = true;
                testPassed = findChessboardCorners(irMat, cv::Size(7, 7),
                                                   corners, flags);
                captureStatus = false;
            } else {
                frameReceived = false;
                captureStatus = false;
                testPassed = false;
            }
        }

        if (frameReceived && testPassed) {
            cv::drawChessboardCorners(irMat, cv::Size(7, 7), corners,
                                      testPassed);
        }

        if (frameReceived) {
            cvui::image(frame, 270, 30, irMat);
        }

        cvui::endColumn();
        cvui::endRow();

        cvui::beginRow(frame, 50, 150);
        cvui::beginColumn(frame, 150, 250);

        if (frameReceived && testPassed) {
            cvui::printf(frame, 30, 230, 1, 0x00ff00, "TEST PASSED");
        }

        if (frameReceived && !testPassed) {
            cvui::printf(frame, 30, 230, 1, 0xff0000, "TEST FAILED:");
            cvui::printf(frame, 40, 260, 0.7, 0xff0000, "INVALID FRAME!");
        }

        if (!frameReceived && !testPassed) {
            cvui::printf(frame, 30, 230, 1, 0xff0000, "TEST FAILED:");
            cvui::printf(frame, 15, 260, 0.7, 0xff0000, "NO FRAME RECEIVED!");
        }

        cvui::trackbar(frame, 30, 340, 750, &targetDistance, (int)300,
                       (int)4500, 1, "%.0Lf", cvui::TRACKBAR_HIDE_VALUE_LABEL,
                       100);
        targetDistance = (targetDistance / 10) * 10;

        cvui::printf(frame, 30, 400, 0.5, 0xffffff, "Target distance:");
        cvui::counter(frame, 50, 420, &targetDistance, 10);

        cvui::printf(frame, 180, 400, 0.5, 0xffffff, "Measured distance:");
        if (measuredDistance != 0) {
            cvui::printf(frame, 220, 420, "Value = %d", measuredDistance);
        }

        if (measuredDistance != 0) {
            if (measuredDistance < targetDistance) {
                precision = (measuredDistance / (float)targetDistance) * 100;
            } else {
                precision = (targetDistance / (float)measuredDistance) * 100;
            }
        }

        cvui::printf(frame, 360, 400, 0.5, 0xffffff, "Precision:");
        cvui::printf(frame, 460, 400, 0.5, 0xffffff, "Temperature:");

        if (measuredDistance != 0) {
            cvui::printf(frame, 355, 420, "Value = %.2f", precision);
        }

        if (temperature != 0) {
            cvui::printf(frame, 465, 420, "Value = %.2f", temperature);
        }

        cvui::endColumn();
        cvui::endRow();

        cvui::update();
        cvui::imshow(WINDOW_NAME, frame);

        cv::waitKey(20);

        /* For the application to start in fullscreen mode the following if must be uncommented */

        /* if (!fullScreen) {
            system("wmctrl -r 'ADITOF-TEST' -b toggle,fullscreen");
            fullScreen = true;
        } */

        if (cv::waitKey(20) == 27) {
            break;
        }
    }

    return 0;
}

int _DisplayIR(cv::Mat *irMat, int *measuredDistance, int targetDistance,
               float *precision, float *temperature) {

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

    int cameraRange = cameraDetails.depthParameters.maxDepth;
    int bitCount = cameraDetails.bitCount;

    const int smallSignalThreshold = 50;
    camera->setControl("noise_reduction_threshold",
                       std::to_string(smallSignalThreshold));

    for (int i = 0; i < 30; i++) {
        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }

        /* Get frame details */
        status = frame.getDetails(frameDetails);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not acquire frame details!";
            return 0;
        }

        int frameHeight = static_cast<int>(frameDetails.height);
        int frameWidth = static_cast<int>(frameDetails.width);

        /* Get distance from center point */
        uint16_t *data;
        frame.getData(aditof::FrameDataType::DEPTH, &data);

        *measuredDistance += static_cast<int>(
            data[frameHeight * frameWidth / 2 + frameWidth / 2]);
    }

    /* Convert frame to IR mat */
    status = fromFrameToIrMat(frame, *irMat);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not convert from frame to mat!";
        return 0;
    }

    /* Convert from frame to depth mat */
    cv::Mat depthMat;
    status = fromFrameToDepthMat(frame, depthMat);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not convert from frame to mat!";
        return 0;
    }

    /* Distance factor */
    double distance_scale = 255.0 / cameraRange;

    /* Convert from raw values to values that opencv can understand */
    depthMat.convertTo(depthMat, CV_8U, distance_scale);

    /* Apply a rainbow color map to the mat to better visualize the
         * depth data */
    applyColorMap(depthMat, depthMat, cv::COLORMAP_RAINBOW);

    int max_value_of_IR_pixel = (1 << bitCount) - 1;

    /* Distance factor IR */
    double distance_scale_ir = 255.0 / max_value_of_IR_pixel;

    irMat->convertTo(*irMat, CV_8U, distance_scale_ir);
    cv::cvtColor(*irMat, *irMat, cv::COLOR_GRAY2RGB);

    *measuredDistance = *measuredDistance / 30;

    if (*measuredDistance != 0) {
        if (*measuredDistance < targetDistance) {
            *precision = (*measuredDistance / (float)targetDistance) * 100;
        } else {
            *precision =
                ((float)targetDistance / (float)*measuredDistance) * 100;
        }
    }

    /* Read the camera temperature */
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
        tempSensors;
    camera->getTemperatureSensors(tempSensors);

    if (tempSensors.empty()) {
        *temperature = 0;
    } else {
        tempSensors[0]->read(*temperature);
    }

    /* Save measured results in a file */
    std::ofstream MyFile("MeasuredResults.txt");
    MyFile << "Target set at distance: " << targetDistance;
    MyFile << "\nCamera measured: " << *measuredDistance;
    MyFile << "\nPrecision: " << *precision;
    MyFile << "\nTemperature: " << *temperature;
    MyFile.close();

    /* Read the camera serial from EEPROM */
    uint8_t eepromSerialID_short[12];
    std::string eepromSerialID = "";
    std::vector<std::shared_ptr<aditof::StorageInterface>> eeproms;
    camera->getEeproms(eeproms);

    if (eeproms.empty()) {
        eepromSerialID = "InvalidEEPROM";
    } else {
        eeproms[0]->read(0x00010016, eepromSerialID_short, 12);
        for (int i = 0; i < 12; i++) {
            eepromSerialID.push_back(eepromSerialID_short[i]);
        }
    }

#ifdef DATA_HANDLING
    saveData(*irMat, depthMat, eepromID);
    moveData(eepromID);
#endif

    return 1;
}

#ifdef DATA_HANDLING
void saveData(cv::Mat irMat, cv::Mat depthMat, std::string eepromID) {

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    bool result = false;
    result = cv::imwrite("irImage.png", irMat, compression_params);
    result = cv::imwrite("depthImage.png", depthMat, compression_params);
}

void moveData(std::string eepromID) {

    std::string filename = "savedResults/";
    int status;

    status = mkdir(filename.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    filename = filename + eepromID;

    status = mkdir(filename.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    char irImage[100] = " sudo mv irImage.png ";
    char depthImage[100] = " sudo mv depthImage.png ";
    char measuredResults[100] = " sudo mv MeasuredResults.txt ";

    strcat(irImage, filename.c_str());
    strcat(depthImage, filename.c_str());
    strcat(measuredResults, filename.c_str());

    system(irImage);
    system(depthImage);
    system(measuredResults);
}
#endif
