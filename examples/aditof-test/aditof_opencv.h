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
#ifndef ADITOF_OPENCV_H
#define ADITOF_OPENCV_H

#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <opencv2/core/core.hpp>

namespace aditof {
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
} // namespace aditof

#endif // ADITOF_OPENCV_H
