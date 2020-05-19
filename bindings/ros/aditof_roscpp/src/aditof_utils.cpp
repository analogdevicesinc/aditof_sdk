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
#include "aditof_utils.h"

#include <aditof/frame.h>
#include <aditof/system.h>

#include <ros/ros.h>

using namespace aditof;
std::shared_ptr<Camera> initCamera(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
    Status status = Status::OK;
    std::string ip;

    if (argc < 2) {
        LOG(INFO) << "No ip provided, attempting to connect to the camera "
                     "through USB";
    } else {
        ip = argv[1];
    }

    System system;
    status = system.initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize system!";
        return nullptr;
    }

    std::vector<std::shared_ptr<Camera>> cameras;
    if (ip.empty()) {
        system.getCameraList(cameras);
    } else {
        system.getCameraListAtIp(cameras, ip);
    }

    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return nullptr;
    }

    std::shared_ptr<Camera> camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return nullptr;
    }
    return camera;
}

void setFrameType(const std::shared_ptr<aditof::Camera> &camera,
                  const std::string &type) {
    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return;
    }

    std::vector<std::string>::iterator it =
        std::find(frameTypes.begin(), frameTypes.end(), type);
    if (it == frameTypes.end()) {
        LOG(ERROR) << "Requested frame type is not available";
        return;
    }

    Status status = Status::OK;
    status = camera->setFrameType(type);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return;
    }
}

void setMode(const std::shared_ptr<aditof::Camera> &camera,
             const std::string &mode) {
    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return;
    }

    std::vector<std::string>::iterator it =
        std::find(modes.begin(), modes.end(), mode);
    if (it == modes.end()) {
        LOG(ERROR) << "Requested mode is not available";
        return;
    }

    Status status = Status::OK;
    status = camera->setMode(mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return;
    }
}

void setCameraRevision(const std::shared_ptr<aditof::Camera> &camera,
                       aditof::Revision rev) {
    auto specifics = camera->getSpecifics();
    auto cam96tof1Specifics =
        std::dynamic_pointer_cast<Camera96Tof1Specifics>(specifics);
    cam96tof1Specifics->setCameraRevision(rev);
}

void applyNoiseReduction(const std::shared_ptr<Camera> &camera, int threshold) {

    auto specifics = camera->getSpecifics();
    auto cam96tof1Specifics =
        std::dynamic_pointer_cast<Camera96Tof1Specifics>(specifics);
    if (cam96tof1Specifics) {
        cam96tof1Specifics->setNoiseReductionThreshold(threshold);
        cam96tof1Specifics->enableNoiseReduction(true);
    } else {
        auto chiconySpecifics =
            std::dynamic_pointer_cast<CameraChiconySpecifics>(specifics);
        if (chiconySpecifics) {
            chiconySpecifics->setNoiseReductionThreshold(threshold);
            chiconySpecifics->enableNoiseReduction(true);
        }
    }
}

void disableNoiseReduction(const std::shared_ptr<Camera> &camera) {

    auto specifics = camera->getSpecifics();
    auto cam96tof1Specifics =
        std::dynamic_pointer_cast<Camera96Tof1Specifics>(specifics);
    if (cam96tof1Specifics) {
        cam96tof1Specifics->enableNoiseReduction(false);
    } else {
        auto chiconySpecifics =
            std::dynamic_pointer_cast<CameraChiconySpecifics>(specifics);
        if (chiconySpecifics) {
            chiconySpecifics->enableNoiseReduction(false);
        }
    }
}

void getNewFrame(const std::shared_ptr<Camera> &camera, aditof::Frame *frame) {
    Status status = Status::OK;
    status = camera->requestFrame(frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
    }
}

uint16_t *getFrameData(aditof::Frame *frame, FrameDataType dataType) {
    uint16_t *frameData;
    Status status = Status::OK;
    status = frame->getData(dataType, &frameData);

    if (status != Status::OK) {
        LOG(ERROR) << "Could not get frame data!";
        return nullptr;
    }

    if (!frameData) {
        LOG(ERROR) << "no memory allocated in frame";
        return nullptr;
    }
    return frameData;
}

IntrinsicParameters getIntrinsics(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.intrinsics;
}

int getRangeMax(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.maxDepth;
}

int getRangeMin(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.minDepth;
}

void irTo16bitGrayscale(uint16_t *frameData, int width, int height) {
    std::vector<uint16_t> data(frameData, frameData + width * height);

    auto min_val = std::min_element(data.begin(), data.end());
    auto max_val = std::max_element(data.begin(), data.end());
    uint16_t delta = *max_val - *min_val;
    int minColorValue = 0;

    for (int i = 0; i < width * height; i++) {
        float norm_val = static_cast<float>(data[i] - *min_val) / delta;
        float grayscale_val =
            norm_val * std::numeric_limits<unsigned short int>::max() +
            (1.0f - norm_val) * minColorValue;
        frameData[i] = static_cast<uint16_t>(grayscale_val);
    }
}
