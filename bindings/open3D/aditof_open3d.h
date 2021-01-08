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
#ifndef ADITOF_OPEN3D_H
#define ADITOF_OPEN3D_H

#include <open3D/Open3D.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>

using namespace open3d;

namespace aditof {
Status fromFrameToDepthImg(Frame &frame, int camera_rangeMin,
                           int camera_rangeMax, geometry::Image &image) {
    FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height);
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *depthData;
    frame.getData(FrameDataType::DEPTH, &depthData);

    if (depthData == nullptr) {
        return Status::GENERIC_ERROR;
    }

    image.Prepare(frameWidth, frameHeight, 1, 1);
    for (int i = 0; i < frameHeight * frameWidth; i++) {
        uint8_t *p =
            static_cast<uint8_t *>(image.data_.data() + i * sizeof(uint8_t));
        int16_t value =
            (*(depthData + i)) * 255.0 / (camera_rangeMax - camera_rangeMin) -
            ((255.0 / (camera_rangeMax - camera_rangeMin)) * camera_rangeMin);
        if (value < 0)
            value = 0;
        *p = static_cast<uint8_t>(value <= 255 ? value : 255);
        p++;
    }

    return Status::OK;
}

Status fromFrameTo16bitsDepth(Frame &frame, geometry::Image &image) {
    FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height);
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *depthData;
    frame.getData(FrameDataType::DEPTH, &depthData);

    if (depthData == nullptr) {
        return Status::GENERIC_ERROR;
    }

    image.Prepare(frameWidth, frameHeight, 1, 2);
    for (int i = 0; i < frameHeight * frameWidth; i++) {
        image.data_[i * 2] = *(depthData + i) & 0xFF;
        image.data_[i * 2 + 1] = *(depthData + i) >> 8;
    }

    return Status::OK;
}

Status fromFrameToIRImg(Frame &frame, int bitCount, geometry::Image &image) {
    FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height);
    const int frameWidth = static_cast<int>(frameDetails.width);
    int max_value_of_IR_pixel = (1 << bitCount) - 1;

    uint16_t *irData;
    frame.getData(FrameDataType::IR, &irData);

    if (irData == nullptr) {
        return Status::GENERIC_ERROR;
    }

    image.Prepare(frameWidth, frameHeight, 1, 1);
    for (int i = 0; i < frameHeight * frameWidth; i++) {
        uint8_t *p =
            static_cast<uint8_t *>(image.data_.data() + i * sizeof(uint8_t));
        uint16_t value = *(irData + i) * 255.0 / max_value_of_IR_pixel;
        *p = static_cast<uint8_t>(value <= 255 ? value : 255);
        p++;
    }

    return Status::OK;
}
} // namespace aditof

#endif // ADITOF_OPEN3D_H
