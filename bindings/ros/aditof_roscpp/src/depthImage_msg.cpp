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
#include "depthImage_msg.h"
using namespace aditof;

DepthImageMsg::DepthImageMsg() {}

DepthImageMsg::DepthImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                             aditof::Frame *frame, std::string encoding,
                             ros::Time tStamp) {
    imgEncoding = encoding;
    FrameDataToMsg(camera, frame, tStamp);
}

void DepthImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                   aditof::Frame *frame, ros::Time tStamp) {
    FrameDetails fDetails;
    frame->getDetails(fDetails);

    setMetadataMembers(fDetails.width, fDetails.height / 2, tStamp);

    uint16_t *frameData = getFrameData(frame, aditof::FrameDataType::DEPTH);
    if (!frameData) {
        LOG(ERROR) << "getFrameData call failed";
        return;
    }

    setDataMembers(camera, frameData);
}

void DepthImageMsg::setMetadataMembers(int width, int height,
                                       ros::Time tStamp) {
    msg.header.stamp = tStamp;
    msg.header.frame_id = "aditof_depth_img";

    msg.width = width;
    msg.height = height;
    msg.encoding = imgEncoding;
    msg.is_bigendian = false;

    int pixelByteCnt = sensor_msgs::image_encodings::bitDepth(imgEncoding) / 8 *
                       sensor_msgs::image_encodings::numChannels(imgEncoding);
    msg.step = width * pixelByteCnt;

    msg.data.resize(msg.step * height);
}

void DepthImageMsg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                   uint16_t *frameData) {
    if (msg.encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) {
        std::vector<uint16_t> depthData(frameData,
                                        frameData + msg.width * msg.height);
        auto min_range = std::min_element(depthData.begin(), depthData.end());

        dataToRGBA8(*min_range, getRangeMax(camera), frameData);
    } else
        ROS_ERROR("Image encoding invalid or not available");
}

void DepthImageMsg::dataToRGBA8(uint16_t min_range, uint16_t max_range,
                                uint16_t *data) {
    uint8_t *msgDataPtr = msg.data.data();
    int32_t delta = static_cast<uint32_t>(max_range - min_range);

    for (unsigned int i = 0; i < msg.width * msg.height; i++) {
        //normalized value
        double norm_val = static_cast<double>(
            static_cast<double>(data[i] - min_range) / delta);
        double hue = norm_val * INDIGO + (1.0f - norm_val) * RED;

        Rgba8Color color = HSVtoRGBA8(hue, SAT, VAL);
        std::memcpy(msgDataPtr, &color, 4);
        msgDataPtr += 4;
    }
}

Rgba8Color DepthImageMsg::HSVtoRGBA8(double hue, double sat, double val) {
    double c = 0.0, m = 0.0, x = 0.0;
    double h = hue / 60.0;

    c = sat * val;
    x = c * (1.0 - std::abs(std::fmod(h, 2) - 1.0));
    m = val - c;

    Rgb32Color rgb32;

    rgb32.r = m;
    rgb32.g = m;
    rgb32.b = m;

    if (h <= 1.0) {
        rgb32.r += c;
        rgb32.g += x;
    } else if (h <= 2.0) {
        rgb32.r += x;
        rgb32.g += c;
    } else if (h <= 3.0) {
        rgb32.g += c;
        rgb32.b += x;
    } else if (h <= 4.0) {
        rgb32.g += x;
        rgb32.b += c;
    } else if (h <= 5.0) {
        rgb32.r += x;
        rgb32.b += c;
    } else if (h <= 6.0) {
        rgb32.r += c;
        rgb32.b += x;
    }

    Rgba8Color rgba8;

    rgba8.r = floor(rgb32.r >= 1.0 ? 255 : rgb32.r * 256.0);
    rgba8.g = floor(rgb32.g >= 1.0 ? 255 : rgb32.g * 256.0);
    rgba8.b = floor(rgb32.b >= 1.0 ? 255 : rgb32.b * 256.0);
    rgba8.a = 0XFF;

    return rgba8;
}

void DepthImageMsg::publishMsg(const ros::Publisher &pub) { pub.publish(msg); }
