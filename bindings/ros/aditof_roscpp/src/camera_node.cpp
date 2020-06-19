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
#include "aditof_roscpp/Aditof_roscppConfig.h"
#include "message_factory.h"
#include <aditof_utils.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "../../../../sdk/include/aditof/camera.h"

using namespace aditof;

void callback(aditof_roscpp::Aditof_roscppConfig &config, uint32_t level,
              std::shared_ptr<Camera> &camera) {

    config.groups.camera_tof.noise_reduction.state = true;

    switch (config.mode) {
    case 0:
        setMode(camera, "near");
        applyNoiseReduction(camera, config.threshold);
        break;
    case 1:
        setMode(camera, "medium");
        applyNoiseReduction(camera, config.threshold);
        break;
    case 2:
        disableNoiseReduction(camera);
        setMode(camera, "far");
        config.threshold = 0;

        //disable the noise reduction, as it is not supported for this mode
        config.groups.camera_tof.noise_reduction.state = false;
        break;
    }

    switch (config.revision) {
    case 0:
        setCameraRevision(camera, "RevB");
        break;
    case 1:
        setCameraRevision(camera, "RevC");
        break;
    }
    setIrGammaCorrection(camera, config.ir_gamma);
}

int main(int argc, char **argv) {

    std::shared_ptr<Camera> camera = initCamera(argc, argv);
    ROS_ASSERT_MSG(camera, "initCamera call failed");

    setFrameType(camera, "depth_ir");
    setMode(camera, "near");

    ros::init(argc, argv, "aditof_camera_node");
    dynamic_reconfigure::Server<aditof_roscpp::Aditof_roscppConfig> server;
    dynamic_reconfigure::Server<
        aditof_roscpp::Aditof_roscppConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2, camera);
    server.setCallback(f);

    //create publishers
    ros::NodeHandle nHandle("aditof_roscpp");
    ros::Publisher pcl_pubisher =
        nHandle.advertise<sensor_msgs::PointCloud2>("aditof_pcloud", 5);
    ROS_ASSERT_MSG(pcl_pubisher, "creating pcl_pubisher failed");

    ros::Publisher depth_img_pubisher =
        nHandle.advertise<sensor_msgs::Image>("aditof_depth", 5);
    ROS_ASSERT_MSG(depth_img_pubisher, "creating depth_img_pubisher failed");

    ros::Publisher ir_img_pubisher =
        nHandle.advertise<sensor_msgs::Image>("aditof_ir", 5);
    ROS_ASSERT_MSG(ir_img_pubisher, "creating ir_img_pubisher failed");

    ros::Publisher camera_info_pubisher =
        nHandle.advertise<sensor_msgs::CameraInfo>("aditof_camera_info", 5);
    ROS_ASSERT_MSG(camera_info_pubisher,
                   "creating camera_info_pubisher failed");

    Frame frame;
    getNewFrame(camera, &frame);

    //create messages
    ros::Time timeStamp = ros::Time::now();
    AditofSensorMsg *pcl_msg = MessageFactory::create(
        camera, &frame, MessageType::sensor_msgs_PointCloud2, timeStamp);
    ROS_ASSERT_MSG(pcl_msg, "pointcloud message creation failed");
    PointCloud2Msg *pclMsg = dynamic_cast<PointCloud2Msg *>(pcl_msg);
    ROS_ASSERT_MSG(pclMsg,
                   "downcast from AditofSensorMsg to PointCloud2Msg failed");

    AditofSensorMsg *depth_img_msg = MessageFactory::create(
        camera, &frame, MessageType::sensor_msgs_DepthImage, timeStamp);
    ROS_ASSERT_MSG(depth_img_msg, "depth_image message creation failed");
    DepthImageMsg *depthImgMsg = dynamic_cast<DepthImageMsg *>(depth_img_msg);
    ROS_ASSERT_MSG(depthImgMsg,
                   "downcast from AditofSensorMsg to DepthImageMsg failed");

    AditofSensorMsg *ir_img_msg = MessageFactory::create(
        camera, &frame, MessageType::sensor_msgs_IRImage, timeStamp);
    ROS_ASSERT_MSG(ir_img_msg, "ir_image message creation failed");
    IRImageMsg *irImgMsg = dynamic_cast<IRImageMsg *>(ir_img_msg);
    ROS_ASSERT_MSG(irImgMsg,
                   "downcast from AditofSensorMsg to IRImageMsg failed");

    AditofSensorMsg *camera_info_msg = MessageFactory::create(
        camera, &frame, MessageType::sensor_msgs_CameraInfo, timeStamp);
    ROS_ASSERT_MSG(camera_info_msg, "camera_info_msg message creation failed");
    CameraInfoMsg *cameraInfoMsg =
        dynamic_cast<CameraInfoMsg *>(camera_info_msg);
    ROS_ASSERT_MSG(cameraInfoMsg,
                   "downcast from AditofSensorMsg to CameraInfoMsg failed");

    while (ros::ok()) {
        ros::Time tStamp = ros::Time::now();
        getNewFrame(camera, &frame);

        pclMsg->FrameDataToMsg(camera, &frame, tStamp);
        pclMsg->publishMsg(pcl_pubisher);

        depthImgMsg->FrameDataToMsg(camera, &frame, tStamp);
        depthImgMsg->publishMsg(depth_img_pubisher);

        irImgMsg->FrameDataToMsg(camera, &frame, tStamp);
        irImgMsg->publishMsg(ir_img_pubisher);

        cameraInfoMsg->FrameDataToMsg(camera, &frame, tStamp);
        cameraInfoMsg->publishMsg(camera_info_pubisher);

        ros::spinOnce();
    }

    delete pcl_msg;
    delete depth_img_msg;
    delete ir_img_msg;
    delete camera_info_msg;
    return 0;
}
