/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif

using namespace aditof;

aditof::Status fromFrameToDepthMat(aditof::Frame &frame, cv::Mat &mat) {
    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height);
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *depthData;
    frame.getData(aditof::FrameDataType::DEPTH, &depthData);

    if (depthData == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    mat = cv::Mat(frameHeight, frameWidth, CV_16UC1, depthData);

    return aditof::Status::OK;
}

aditof::Status fromFrameToIrMat(aditof::Frame &frame, cv::Mat &mat) {
    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height);
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *irData;
    frame.getData(aditof::FrameDataType::IR, &irData);

    if (irData == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    mat = cv::Mat(frameHeight, frameWidth, CV_16UC1, irData);

    return aditof::Status::OK;
}

int main(int argc, char *argv[]) {
#ifndef JS_BINDINGS 
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
#endif

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

    status = camera->setFrameType("depth_ir");
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

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    int cameraRange = cameraDetails.depthParameters.maxDepth;
    aditof::Frame frame;

    const int smallSignalThreshold = 50;
    camera->setControl("noise_reduction_threshold",
                       std::to_string(smallSignalThreshold));

    cv::namedWindow("Display Depth", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Display Ir", cv::WINDOW_AUTOSIZE);

    while (cv::waitKey(1) != 27 &&
           getWindowProperty("Display Depth", cv::WND_PROP_AUTOSIZE) >= 0) {

        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }

        /* Convert from frame to depth mat */
        cv::Mat mat_depth;
        status = fromFrameToDepthMat(frame, mat_depth);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return 0;
        }

        /* Convert from frame to ir mat */
        cv::Mat mat_ir;
        status = fromFrameToIrMat(frame, mat_ir);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return 0;
        }

        /* Distance factor */
        double distance_scale = 255.0 / cameraRange;

        /* Convert from raw values to values that opencv can understand */
        mat_depth.convertTo(mat_depth, CV_8U, distance_scale);

        /* Apply a rainbow color map to the mat to better visualize the
         * depth data */
        applyColorMap(mat_depth, mat_depth, cv::COLORMAP_RAINBOW);

        /* Display the image */
        imshow("Display Depth", mat_depth);
        imshow("Display Ir", mat_ir);
    }

    return 0;
}
