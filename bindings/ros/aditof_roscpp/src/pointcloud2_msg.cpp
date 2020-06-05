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
#include "pointcloud2_msg.h"
using namespace aditof;

PointCloud2Msg::PointCloud2Msg() {}

PointCloud2Msg::PointCloud2Msg(const std::shared_ptr<aditof::Camera> &camera,
                               aditof::Frame *frame, ros::Time tStamp) {
    FrameDataToMsg(camera, frame, tStamp);
}

void PointCloud2Msg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                    aditof::Frame *frame, ros::Time tStamp) {
    FrameDetails fDetails;
    frame->getDetails(fDetails);

    setMetadataMembers(fDetails.width, fDetails.height / 2, tStamp);
    setDataMembers(camera, frame);
}

void PointCloud2Msg::setMetadataMembers(int width, int height,
                                        ros::Time tStamp) {
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                  1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1,
                                  sensor_msgs::PointField::UINT16);
    msg.header.stamp = tStamp;
    msg.header.frame_id = "base_link";

    msg.width = width;
    msg.height = height;

    msg.is_bigendian = false;
    msg.point_step = 3 * sizeOfPointField(sensor_msgs::PointField::FLOAT32) +
                     sizeOfPointField(sensor_msgs::PointField::UINT16);
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;

    msg.data.resize(width * height * msg.point_step);
}

void PointCloud2Msg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                    aditof::Frame *frame) {
    IntrinsicParameters intr = getIntrinsics(camera);

    float fx = intr.cameraMatrix[0];
    float fy = intr.cameraMatrix[4];
    float x0 = intr.cameraMatrix[2];
    float y0 = intr.cameraMatrix[5];

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_intensity(msg, "intensity");

    const int frameHeight = static_cast<int>(msg.height);
    const int frameWidth = static_cast<int>(msg.width);

    uint16_t *frameDataDepth =
        getFrameData(frame, aditof::FrameDataType::DEPTH);
    uint16_t *frameDataIR = getFrameData(frame, aditof::FrameDataType::IR);

    irTo16bitGrayscale(frameDataIR, frameWidth, frameHeight);

    for (int i = 0; i < frameHeight; i++) {
        for (int j = 0; j < frameWidth;
             j++, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            int index = i * msg.width + j;
            float z = static_cast<float>(frameDataDepth[index]) / 1000.0f;

            *iter_x = z * (j - x0) / fx;
            *iter_y = z * (i - y0) / fy;
            *iter_z = z;

            *iter_intensity = frameDataIR[index];
        }
    }
}

void PointCloud2Msg::publishMsg(const ros::Publisher &pub) { pub.publish(msg); }
