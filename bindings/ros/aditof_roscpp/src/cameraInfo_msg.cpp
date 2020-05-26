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
#include "cameraInfo_msg.h"

using namespace aditof;

CameraInfoMsg::CameraInfoMsg() {}

CameraInfoMsg::CameraInfoMsg(const std::shared_ptr<aditof::Camera> &camera,
                             aditof::Frame *frame, ros::Time tStamp) {
    FrameDataToMsg(camera, frame, tStamp);
}

void CameraInfoMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                   aditof::Frame *frame, ros::Time tStamp) {
    FrameDetails fDetails;
    frame->getDetails(fDetails);

    setMembers(camera, fDetails.width, fDetails.height / 2, tStamp);
}

void CameraInfoMsg::setMembers(const std::shared_ptr<Camera> &camera, int width,
                               int height, ros::Time tStamp) {
    msg.header.stamp = tStamp;
    msg.header.frame_id = "aditof_camera_info";

    msg.width = width;
    msg.height = height;
    msg.distortion_model = "plumb_bob";

    IntrinsicParameters intr = getIntrinsics(camera);

    msg.D = std::vector<double>(intr.distCoeffs.begin(), intr.distCoeffs.end());
    float *ptr = intr.cameraMatrix.data();
    msg.K = {ptr[0], ptr[1], ptr[2], ptr[3], ptr[4],
             ptr[5], ptr[6], ptr[7], ptr[8]};
    msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg.P = {msg.K[0], msg.K[1], msg.K[2], 0.0f,     msg.K[3], msg.K[4],
             msg.K[5], 0.0f,     msg.K[6], msg.K[7], msg.K[8], 0.0f};
}

void CameraInfoMsg::publishMsg(const ros::Publisher &pub) { pub.publish(msg); }
