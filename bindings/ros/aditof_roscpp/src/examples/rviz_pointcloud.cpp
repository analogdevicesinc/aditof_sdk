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
#include "message_factory.h"
#include <aditof_utils.h>
#include <ros/ros.h>

using namespace aditof;

int main(int argc, char **argv) {

    std::shared_ptr<Camera> camera = initCamera(argc, argv);
    if (!camera) {
        ROS_ERROR("initCamera call failed");
        return -1;
    }

    std::vector<std::string> availableFrameTypes;
    getAvailableFrameType(camera, availableFrameTypes);
    if (availableFrameTypes.front().find("rgb") != std::string::npos) {
        setFrameType(camera, "depth_ir_rgb");
    } else {
        setFrameType(camera, "depth_ir");
    }
    setMode(camera, "medium");

    ros::init(argc, argv, "aditof_rviz_node");

    ros::NodeHandle nHandle;
    ros::Publisher frame_pubisher =
        nHandle.advertise<sensor_msgs::PointCloud2>("aditof_pcloud", 100);

    applyNoiseReduction(camera, 0);

    Frame frame;
    getNewFrame(camera, &frame);

    AditofSensorMsg *msg = MessageFactory::create(
        camera, &frame, MessageType::sensor_msgs_PointCloud2, ros::Time::now());

    if (!msg) {
        ROS_ERROR("pointcloud message creation failed");
        return -1;
    }

    while (ros::ok()) {
        getNewFrame(camera, &frame);
        PointCloud2Msg *pclMsg = dynamic_cast<PointCloud2Msg *>(msg);

        if (!pclMsg) {
            ROS_ERROR("downcast from AditofSensorMsg to PointCloud2Msg failed");
            return -1;
        }
        pclMsg->FrameDataToMsg(camera, &frame, ros::Time::now());
        pclMsg->publishMsg(frame_pubisher);
    }

    delete msg;
    return 0;
}
