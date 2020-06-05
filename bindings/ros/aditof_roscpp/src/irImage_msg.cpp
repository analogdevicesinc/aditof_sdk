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
#include "irImage_msg.h"
using namespace aditof;

IRImageMsg::IRImageMsg() {}

IRImageMsg::IRImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                       aditof::Frame *frame, std::string encoding,
                       ros::Time tStamp) {
    imgEncoding = encoding;
    FrameDataToMsg(camera, frame, tStamp);
}

void IRImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                aditof::Frame *frame, ros::Time tStamp) {
    FrameDetails fDetails;
    frame->getDetails(fDetails);

    setMetadataMembers(fDetails.width, fDetails.height / 2, tStamp);

    uint16_t *frameData = getFrameData(frame, aditof::FrameDataType::IR);
    if (!frameData) {
        LOG(ERROR) << "getFrameData call failed";
        return;
    }

    setDataMembers(camera, frameData);
}

void IRImageMsg::setMetadataMembers(int width, int height, ros::Time tStamp) {
    msg.header.stamp = tStamp;
    msg.header.frame_id = "aditof_ir_img";

    msg.width = width;
    msg.height = height;
    msg.encoding = imgEncoding;
    msg.is_bigendian = false;

    int pixelByteCnt = sensor_msgs::image_encodings::bitDepth(imgEncoding) / 8 *
                       sensor_msgs::image_encodings::numChannels(imgEncoding);
    msg.step = width * pixelByteCnt;

    msg.data.resize(msg.step * height);
}

void IRImageMsg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                uint16_t *frameData) {
    if (msg.encoding.compare(sensor_msgs::image_encodings::MONO16) == 0) {
        irTo16bitGrayscale(frameData, msg.width, msg.height);
        uint8_t *msgDataPtr = msg.data.data();
        std::memcpy(msgDataPtr, frameData, msg.step * msg.height);
    } else
        ROS_ERROR("Image encoding invalid or not available");
}

void IRImageMsg::publishMsg(const ros::Publisher &pub) { pub.publish(msg); }
