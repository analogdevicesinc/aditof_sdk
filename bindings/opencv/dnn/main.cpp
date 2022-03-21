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
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
// #include <opencv2/ximgproc/edge_filter.hpp>
#include <aditof/depth_sensor_interface.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>

#include "../aditof_opencv.h"

const size_t inWidth = 300;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;
const char *classNames[] = {
    "background", "aeroplane",   "bicycle", "bird",  "boat",
    "bottle",     "bus",         "car",     "cat",   "chair",
    "cow",        "diningtable", "dog",     "horse", "motorbike",
    "person",     "pottedplant", "sheep",   "sofa",  "train",
    "tvmonitor"};

int main(int argc, char *argv[]) {

    using namespace aditof;

    cv::dnn::Net net = cv::dnn::readNetFromCaffe(PROTOTXT, MODEL);

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
        return -1;
    }

    auto camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return -1;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return -1;
    }

    status = camera->setFrameType(frameTypes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return -1;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return -1;
    }

    status = camera->setMode(modes[0]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return -1;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    int cameraRange = cameraDetails.depthParameters.maxDepth;
    int bitCount = cameraDetails.bitCount;

    aditof::Frame frame;
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return -1;
    }

    const int smallSignalThreshold = 50;
    camera->setControl("noise_reduction_threshold",
                       std::to_string(smallSignalThreshold));

    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    cv::namedWindow("Display Objects Depth and IR", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Display Objects Depth", cv::WINDOW_AUTOSIZE);

    cv::Size cropSize;
    if ((float)frameDetails.width / (float)(frameDetails.height) > WHRatio) {
        cropSize =
            cv::Size(static_cast<int>((float)(frameDetails.height) * WHRatio),
                     (frameDetails.height));
    } else {
        cropSize =
            cv::Size(frameDetails.width,
                     static_cast<int>((float)frameDetails.width / WHRatio));
    }

    cv::Rect crop(cv::Point(frameDetails.width - cropSize.width,
                            frameDetails.height - cropSize.height),
                  cropSize);

    // Look up table to adjust image => use gamma correction
    float gamma = 0.4f;
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar *p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);

    while (cv::waitKey(1) != 27 &&
           getWindowProperty("Display Objects Depth", cv::WND_PROP_AUTOSIZE) >=
               0) {
        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return -1;
        }

        /* Obtain the depth mat from the frame, this will be used for distance
         * calculation*/
        cv::Mat frameMat;
        status = fromFrameToDepthMat(frame, frameMat);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return -1;
        }

        /* Obtain the depth mat from the frame */
        cv::Mat depthMat;
        status = fromFrameToDepthMat(frame, depthMat);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return -1;
        }

        /* Obtain the ir mat from the frame */
        cv::Mat irMat;
        status = fromFrameToIrMat(frame, irMat);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return -1;
        }

        /* Distance factor */
        double distance_scale = 255.0 / cameraRange;

        int max_value_of_IR_pixel = (1 << bitCount) - 1;

        /* Distance factor IR */
        double distance_scale_ir = 255.0 / max_value_of_IR_pixel;

        /* Convert from raw values to values that opencv can understand */
        frameMat.convertTo(frameMat, CV_8U, distance_scale);
        applyColorMap(frameMat, frameMat, cv::COLORMAP_RAINBOW);

        irMat.convertTo(irMat, CV_8U, distance_scale_ir);
        cv::cvtColor(irMat, irMat, cv::COLOR_GRAY2RGB);

        /* Use a combination between ir mat & depth mat as input for the Net as
         * we currently don't have an rgb source */
        cv::Mat resultMat;
        cv::addWeighted(irMat, 0.4, frameMat, 0.6, 0, resultMat);

        cv::Mat inputBlob =
            cv::dnn::blobFromImage(resultMat, inScaleFactor,
                                   cv::Size(inWidth, inHeight), meanVal, false);

        net.setInput(inputBlob, "data");

        cv::Mat detection = net.forward("detection_out");

        cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F,
                             detection.ptr<float>());

        frameMat = frameMat(crop);
        resultMat = resultMat(crop);

        float confidenceThreshold = 0.4f;

        for (int i = 0; i < detectionMat.rows; i++) {
            float confidence = detectionMat.at<float>(i, 2);

            if (confidence > confidenceThreshold) {
                size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));

                int xLeftBottom = static_cast<int>(
                    detectionMat.at<float>(i, 3) * frameMat.cols);
                int yLeftBottom = static_cast<int>(
                    detectionMat.at<float>(i, 4) * frameMat.rows);
                int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) *
                                                 frameMat.cols);
                int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) *
                                                 frameMat.rows);

                cv::Rect object((int)xLeftBottom, (int)yLeftBottom,
                                (int)(xRightTop - xLeftBottom),
                                (int)(yRightTop - yLeftBottom));

                object = object & cv::Rect(0, 0, frameMat.cols, frameMat.rows);

                auto center = (object.br() + object.tl()) * 0.5;

                cv::drawMarker(frameMat, center, cv::Scalar(0, 0, 0),
                               cv::MARKER_CROSS);
                cv::drawMarker(resultMat, center, cv::Scalar(0, 0, 0),
                               cv::MARKER_CROSS);

                std::ostringstream ss;
                ss.str("");
                ss << std::setprecision(3)
                   << depthMat.at<ushort>(center) / 1000.0 << " meters";
                cv::String depth_string(ss.str());

                std::ostringstream ss_conf;
                ss_conf.str("");
                ss_conf << "Confidence: " << std::setprecision(4) << confidence;
                cv::String conf_string(ss_conf.str());

                cv::rectangle(frameMat, object, cv::Scalar(0, 255, 0));
                cv::rectangle(resultMat, object, cv::Scalar(0, 255, 0));
                cv::String label_depth =
                    cv::String(classNames[objectClass]) + ": " + depth_string;

                cv::String label_conf = conf_string;
                int baseLine = 0;
                cv::Size labelSize_depth = getTextSize(
                    label_depth, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::Size labelSize_conf = getTextSize(
                    label_conf, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::Size labelSize =
                    labelSize_depth.width > labelSize_conf.width
                        ? labelSize_depth
                        : labelSize_conf;

                center.x = center.x - labelSize.width / 2;
                auto conf_position = center;
                conf_position.y = yLeftBottom + 2 * labelSize.height + baseLine;

                cv::rectangle(
                    frameMat,
                    cv::Rect(cv::Point(center.x, yLeftBottom),
                             cv::Size(labelSize.width,
                                      2 * labelSize.height + baseLine * 2)),
                    cv::Scalar(255, 255, 255), cv::FILLED);
                cv::putText(frameMat, label_depth,
                            cv::Point(center.x, yLeftBottom + labelSize.height),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

                cv::putText(frameMat, label_conf, conf_position,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

                cv::rectangle(
                    resultMat,
                    cv::Rect(cv::Point(center.x, yLeftBottom),
                             cv::Size(labelSize.width,
                                      2 * labelSize.height + baseLine * 2)),
                    cv::Scalar(255, 255, 255), cv::FILLED);
                cv::putText(resultMat, label_depth,
                            cv::Point(center.x, yLeftBottom + labelSize.height),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                cv::putText(resultMat, label_conf, conf_position,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
        }

        /* Display the images */
        cv::imshow("Display Objects Depth", frameMat);
        cv::imshow("Display Objects Depth and IR", resultMat);
    }

    return 0;
}
