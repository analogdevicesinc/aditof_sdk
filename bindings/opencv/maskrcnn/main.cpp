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

#include <fstream>
#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include <time.h>

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

#include "../aditof_opencv.h"

const char *keys =
    "{model         |<none>| Path to a .pb file with weights.   }"
    "{config        |<none>| Path to a .pxtxt file contains network "
    "configuration.   }"
    "{classes       |<none>| Path to a text file with names of classes.   }";
using namespace cv;
using namespace dnn;
using namespace std;
using namespace aditof;

/* Initialize the parameters for MaskR-CNN algorithm */
float confThreshold = 0.5;
float maskThreshold = 0.3;

vector<string> classes;
vector<Scalar> colors;
Net net;
bool start_thread = true;

void drawBox(const Mat &distance_map, Mat &frame, int classId, float conf,
             Rect box, Mat &objectMask);

void postprocess(const Mat &distance_map, Mat &frame, const vector<Mat> &outs);

void rgb_color_code(int rgb[]);

void apply_maskrcnn_algorithm(Mat distance_map, Mat image);

int main(int argc, char **argv) {
    CommandLineParser parser(argc, argv, keys);
    parser.about("Use this script to run object detection using OpenCV.");

    /* Give the configuration and weight files for the model */
    string classesFile = "mscoco_labels.names";
    string textGraph = "mask_rcnn_inception_v2_coco_2018_01_28.pbtxt";
    string modelWeights = "frozen_inference_graph.pb";

    if (parser.has("classes")) {
        classesFile = parser.get<string>("classes");
        LOG(INFO) << "Path to classes: " << classesFile;
    }
    if (parser.has("config")) {
        textGraph = parser.get<string>("config");
        LOG(INFO) << "Path to configuration: " << textGraph;
    }
    if (parser.has("model")) {
        modelWeights = parser.get<string>("model");
        LOG(INFO) << "Path to model: " << modelWeights;
    }

    /* Load names of classes */
    if (!classesFile.empty() && classesFile.back() == '\n')
        classesFile.pop_back();
    ifstream ifs(classesFile);
    if (!ifs.is_open()) {
        LOG(ERROR) << "Please give the correct location of the classes file";
        return -1;
    }

    string line;
    while (getline(ifs, line)) {
        classes.push_back(line);
    }

    srand(time(0));
    uint16_t numberOfColors = 20;
    /* Generate 20 colors to work with */
    for (int c = 0; c < numberOfColors; c++) {
        int rgb[3];
        rgb_color_code(rgb);
        Scalar color = Scalar(rgb[0], rgb[1], rgb[2], 255.0);
        colors.push_back(color);
    }

    /* Load the network */
    try {
        net = readNetFromTensorflow(modelWeights, textGraph);
    } catch (...) {
        LOG(ERROR) << "Please give the correct location of the model and "
                      "configuration";
        return -1;
    }

    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

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

    /* Distance factor */
    int cameraRange = cameraDetails.depthParameters.maxDepth;
    double distance_scale = 255.0 / cameraRange;

    /* Distance factor IR */
    int bitCount = cameraDetails.bitCount;
    int max_value_of_IR_pixel = (1 << bitCount) - 1;
    double distance_scale_ir = 255.0 / max_value_of_IR_pixel;

    aditof::Frame frame;
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return -1;
    }

    const int smallSignalThreshold = 70;
    camera->setControl("noise_reduction_threshold",
                       std::to_string(smallSignalThreshold));

    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    cv::namedWindow("Color image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mask-RCNN in OpenCV", cv::WINDOW_AUTOSIZE);

    int frameCount = 0;
    int displayFps = 0;
    auto startTime = std::chrono::system_clock::now();
    std::thread thread_maskrcnn;

    while (cv::waitKey(1) != 27 &&
           getWindowProperty("Color image", cv::WND_PROP_AUTOSIZE) >= 0) {
        frameCount++;

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

        /* Obtain the distance mat from the frame */
        cv::Mat distanceMat;
        status = fromFrameToDepthMat(frame, distanceMat);
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

        /* Convert from raw values to values that opencv can understand */
        frameMat.convertTo(frameMat, CV_8U, distance_scale);
        applyColorMap(frameMat, frameMat, cv::COLORMAP_RAINBOW);

        irMat.convertTo(irMat, CV_8U, distance_scale_ir);
        cv::cvtColor(irMat, irMat, cv::COLOR_GRAY2RGB);

        /* Use a combination between ir mat & depth mat as input for the Net as
         * we currently don't have an rgb source */
        cv::Mat resultMat, colorImage;
        cv::addWeighted(irMat, 0.4, frameMat, 0.6, 0, resultMat);
        cv::addWeighted(irMat, 0.4, frameMat, 0.6, 0, colorImage);

        /* Calculate FPS */
        auto endTime = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = endTime - startTime;
        if (elapsed.count() >= 1) {
            displayFps = static_cast<int>(frameCount / elapsed.count());
            frameCount = 0;
            startTime = endTime;
        }
        string label = format("FPS: %i", displayFps);

        /* Display the label */
        int baseLine;
        Size labelSize =
            getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        rectangle(colorImage,
                  Point(10, colorImage.rows - 10 - labelSize.height -
                                int(baseLine * 0.5)),
                  Point(10 + labelSize.width, colorImage.rows - 10 + baseLine),
                  Scalar(255, 255, 255), FILLED);

        putText(colorImage, label, Point(10, colorImage.rows - 10),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));

        /* Display the color image */
        cv::imshow("Color image", colorImage);

        /* Create and start thread that does the maskR-CNN algorithm */
        if (start_thread == true) {
            if (thread_maskrcnn.joinable()) {
                thread_maskrcnn.join();
            }
            thread_maskrcnn =
                thread(apply_maskrcnn_algorithm, distanceMat, resultMat);
            start_thread = false;
        }
    }

    /* Make sure the thread has finished before closing the program */
    if (thread_maskrcnn.joinable()) {
        thread_maskrcnn.join();
    }
    return 0;
}

void apply_maskrcnn_algorithm(Mat distance_map, Mat image) {
    auto startTime = std::chrono::system_clock::now();

    /* Create a 4D blob from a frame */
    Mat blob;

    blobFromImage(image, blob, 1.0, Size(image.cols, image.rows), Scalar(),
                  true, false);

    /* Sets the input to the network */
    net.setInput(blob);

    /* Runs the forward pass to get output from the output layers */
    std::vector<String> outNames(2);
    outNames[0] = "detection_out_final";
    outNames[1] = "detection_masks";
    vector<Mat> outs;
    net.forward(outs, outNames);

    /* Extract the bounding box and mask for each of the detected objects */
    postprocess(distance_map, image, outs);

    /* Calculate the time for computing the algorithm */
    auto endTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    string label = format("Time per frame: %.2fs", elapsed.count());

    /* Display the label */
    int baseLine;
    Size labelSize =
        getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    rectangle(
        image,
        Point(10, image.rows - 10 - labelSize.height - int(baseLine * 0.5)),
        Point(10 + labelSize.width, image.rows - 10 + baseLine),
        Scalar(255, 255, 255), FILLED);

    putText(image, label, Point(10, image.rows - 10), FONT_HERSHEY_SIMPLEX, 0.5,
            Scalar(0, 0, 0));

    /* Display the color image with the detected objects */
    cv::imshow("Mask-RCNN in OpenCV", image);
    start_thread = true;
}

/* Create a random color */
void rgb_color_code(int rgb[]) {
    for (int i = 0; i < 3; i++) {
        rgb[i] = rand() % 256;
    }
}

/* For each frame, extract the bounding box and mask for each detected object */
void postprocess(const Mat &distance_map, Mat &frame, const vector<Mat> &outs) {
    Mat outDetections = outs[0];
    Mat outMasks = outs[1];

    const int numDetections = outDetections.size[2];
    const int numClasses = outMasks.size[1];

    outDetections = outDetections.reshape(1, outDetections.total() / 7);
    for (int i = 0; i < numDetections; ++i) {
        float score = outDetections.at<float>(i, 2);
        if (score > confThreshold) {
            /* Extract the bounding box */
            int classId = static_cast<int>(outDetections.at<float>(i, 1));
            int left =
                static_cast<int>(frame.cols * outDetections.at<float>(i, 3));
            int top =
                static_cast<int>(frame.rows * outDetections.at<float>(i, 4));
            int right =
                static_cast<int>(frame.cols * outDetections.at<float>(i, 5));
            int bottom =
                static_cast<int>(frame.rows * outDetections.at<float>(i, 6));

            left = max(0, min(left, frame.cols - 1));
            top = max(0, min(top, frame.rows - 1));
            right = max(0, min(right, frame.cols - 1));
            bottom = max(0, min(bottom, frame.rows - 1));
            Rect box = Rect(left, top, right - left + 1, bottom - top + 1);

            /* Extract the mask for the object */
            Mat objectMask(outMasks.size[2], outMasks.size[3], CV_32F,
                           outMasks.ptr<float>(i, classId));

            /* Draw bounding box, colorize and show the mask on the image */
            drawBox(distance_map, frame, classId, score, box, objectMask);
        }
    }
}

/* Draw the predicted bounding box, colorize and show the mask on the image */
void drawBox(const Mat &distance_map, Mat &frame, int classId, float conf,
             Rect box, Mat &objectMask) {
    /* Select a color */
    Scalar color = colors[classId % colors.size()];

    /* Draw a rectangle displaying the bounding box */
    rectangle(frame, Point(box.x, box.y),
              Point(box.x + box.width, box.y + box.height), color, 3);

    /* Resize the mask, threshold, color and apply it on the image */
    resize(objectMask, objectMask, Size(box.width, box.height));
    Mat mask = (objectMask > maskThreshold);
    Mat coloredRoi = (0.7 * color + 0.3 * frame(box));
    coloredRoi.convertTo(coloredRoi, CV_8UC3);
    mask.convertTo(mask, CV_8U);
    coloredRoi.copyTo(frame(box), mask);

    /* Calculate the distance to the object */
    Mat dist_obj =
        Mat::zeros(Size(distance_map.cols, distance_map.rows), CV_8U);
    Mat roi_dist = distance_map(box);
    roi_dist.copyTo(dist_obj, mask);
    Scalar tempVal = mean(dist_obj, mask);
    float meanDistance = tempVal.val[0];

    /* Create the predicted label, the distance and associated probability of detection */
    string label_dist = format("%.4f", (meanDistance / 1000.0));
    if (!classes.empty()) {
        CV_Assert(classId < (int)classes.size());
        label_dist = classes[classId] + ": " + label_dist + " meters";
    }

    string label_conf = format("Confidence: %.4f", conf);

    int baseLine_dist;
    Size labelSize_dist =
        getTextSize(label_dist, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine_dist);

    int baseLine_conf;
    Size labelSize_conf =
        getTextSize(label_conf, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine_conf);

    int baseLine;
    Size labelSize;

    if (labelSize_dist.width > labelSize_conf.width) {
        labelSize = labelSize_dist;
        baseLine = baseLine_dist;
    } else {
        labelSize = labelSize_conf;
        baseLine = baseLine_conf;
    }

    /* Display the label at the top of the bounding box, in the middle */
    rectangle(frame,
              Point(int((box.x + box.width) * 0.5) - int(labelSize.width * 0.5),
                    box.y + baseLine),
              Point(int((box.x + box.width) * 0.5) + int(labelSize.width * 0.5),
                    box.y + 3 * baseLine + 2 * labelSize.height),
              Scalar(255, 255, 255), FILLED);
    putText(frame, label_dist,
            Point(int((box.x + box.width) * 0.5) - int(labelSize.width * 0.5),
                  box.y + labelSize.height + baseLine),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    putText(frame, label_conf,
            Point(int((box.x + box.width) * 0.5) - int(labelSize.width * 0.5),
                  box.y + 2 * labelSize.height + 2 * baseLine),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
}
