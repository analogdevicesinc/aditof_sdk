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

#include "../aditof_open3d.h"

static const uint8_t colormap[3 * 256] = {
#include "colormap.txt"
};

using namespace aditof;

int main(int argc, char *argv[]) {
#ifndef JS_BINDINGS 
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
#endif

    Status status = Status::OK;

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraList(cameras);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found!";
        return 0;
    }

    auto camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return 0;
    }

    status = camera->setFrameType(frameTypes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return 0;
    }

    status = camera->setMode(modes[0]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    aditof::Frame frame;

    /* Get the camera details */
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    int camera_rangeMax = cameraDetails.depthParameters.maxDepth;
    int camera_rangeMin = cameraDetails.depthParameters.minDepth;
    int bitCount = cameraDetails.bitCount;

    aditof::IntrinsicParameters intrinsics = cameraDetails.intrinsics;
    double fx = intrinsics.cameraMatrix.at(0);
    double fy = intrinsics.cameraMatrix.at(4);
    double cx = intrinsics.cameraMatrix.at(2);
    double cy = intrinsics.cameraMatrix.at(5);

    /* Enable noise reduction for better results */
    const int smallSignalThreshold = 100;
    camera->setControl("noise_reduction_threshold",
                       std::to_string(smallSignalThreshold));

    /* Request frame from camera */
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 0;
    }
    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);
    int frameHeight = static_cast<int>(frameDetails.height);
    int frameWidth = static_cast<int>(frameDetails.width);

    /* Create visualizer for depth and IR images */
    auto visualized_ir_img = std::make_shared<geometry::Image>();
    visualized_ir_img->Prepare(frameWidth, frameHeight, 1, 1);
    visualization::Visualizer ir_vis;
    ir_vis.CreateVisualizerWindow("IR Image", 2 * frameWidth, 2 * frameHeight);
    bool is_geometry_added_ir = false;

    auto visualized_depth_img = std::make_shared<geometry::Image>();
    visualized_depth_img->Prepare(frameWidth, frameHeight, 3, 1);
    visualization::Visualizer depth_vis;
    depth_vis.CreateVisualizerWindow("Depth Image", 2 * frameWidth,
                                     2 * frameHeight);
    bool is_geometry_added_depth = false;

    /* Create visualizer for pointcloud */
    visualization::Visualizer pointcloud_vis;
    pointcloud_vis.CreateVisualizerWindow("Pointcloud", 1200, 1200);
    bool is_geometry_added_pointcloud = false;

    const float PI_VALUE = 3.14;
    std::shared_ptr<geometry::PointCloud> pointcloud_ptr = nullptr;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = static_cast<Eigen::Matrix3d>(
        Eigen::AngleAxisd(PI_VALUE, Eigen::Vector3d::UnitX()));
    camera::PinholeCameraIntrinsic intrinsicParameters(frameWidth, frameHeight,
                                                       fx, fy, cx, cy);

    bool is_window_closed = true;
    while (true) {
        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }
        geometry::Image depth_image;
        status = fromFrameToDepthImg(frame, camera_rangeMin, camera_rangeMax,
                                     depth_image);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to Image!";
        }

        geometry::Image depth16bits_image;
        status = fromFrameTo16bitsDepth(frame, depth16bits_image);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to Image!";
        }

        geometry::Image ir_image;
        status = fromFrameToIRImg(frame, bitCount, ir_image);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to Image!";
        }

        geometry::Image depth_color;
        depth_color.Prepare(frameWidth, frameHeight, 3, 1);
        for (int i = 0; i < frameHeight * frameWidth; i++) {
            memcpy(depth_color.data_.data() + i * 3,
                   &colormap[depth_image.data_[i] * 3], 3);
        }

        geometry::Image ir_color;
        ir_color.Prepare(frameWidth, frameHeight, 3, 1);
        for (int i = 0, j = 0; i < frameHeight * frameWidth; i++, j = j + 3) {
            ir_color.data_[j] = ir_image.data_[i];
            ir_color.data_[j + 1] = ir_image.data_[i];
            ir_color.data_[j + 2] = ir_image.data_[i];
        }

        geometry::Image color_image;
        color_image.Prepare(frameWidth, frameHeight, 3, 1);
        for (int i = 0; i < frameHeight * frameWidth * 3; i = i + 3) {
            color_image.data_[i] =
                ir_color.data_[i] * 0.5 + depth_color.data_[i] * 0.5;
            color_image.data_[i + 1] =
                ir_color.data_[i + 1] * 0.5 + depth_color.data_[i + 1] * 0.5;
            color_image.data_[i + 2] =
                ir_color.data_[i + 2] * 0.5 + depth_color.data_[i + 2] * 0.5;
        }

        visualized_ir_img->data_ = ir_image.data_;
        if (!is_geometry_added_ir) {
            ir_vis.AddGeometry(visualized_ir_img);
            is_geometry_added_ir = true;
        }

        visualized_depth_img->data_ = depth_color.data_;
        if (!is_geometry_added_depth) {
            depth_vis.AddGeometry(visualized_depth_img);
            is_geometry_added_depth = true;
        }
        ir_vis.UpdateGeometry();
        ir_vis.PollEvents();
        ir_vis.UpdateRender();

        depth_vis.UpdateGeometry();
        depth_vis.PollEvents();
        depth_vis.UpdateRender();

        /* create and show pointcloud */
        auto rgbd_ptr = geometry::RGBDImage::CreateFromColorAndDepth(
            color_image, depth16bits_image, 1000.0, 3.0, false);
        if (!pointcloud_ptr) {
            pointcloud_ptr = geometry::PointCloud::CreateFromRGBDImage(
                *rgbd_ptr, intrinsicParameters);

            auto bounding_box = pointcloud_ptr->GetAxisAlignedBoundingBox();
            Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
            trans_to_origin.block<3, 1>(0, 3) = bounding_box.GetCenter() * -1.0;

            pointcloud_ptr->Transform(trans_to_origin.inverse() *
                                      transformation * trans_to_origin);
        } else {
            auto temp = geometry::PointCloud::CreateFromRGBDImage(
                *rgbd_ptr, intrinsicParameters);

            auto bounding_box = temp->GetAxisAlignedBoundingBox();
            Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
            trans_to_origin.block<3, 1>(0, 3) = bounding_box.GetCenter() * -1.0;

            temp->Transform(trans_to_origin.inverse() * transformation *
                            trans_to_origin);

            pointcloud_ptr->points_ = temp->points_;
            pointcloud_ptr->normals_ = temp->normals_;
            pointcloud_ptr->colors_ = temp->colors_;
        }

        if (!is_geometry_added_pointcloud) {
            pointcloud_vis.AddGeometry(pointcloud_ptr);
            is_geometry_added_pointcloud = true;
        }

        if (is_window_closed == false) {
            pointcloud_vis.DestroyVisualizerWindow();
            ir_vis.DestroyVisualizerWindow();
            depth_vis.DestroyVisualizerWindow();
            break;
        } else {
            pointcloud_vis.UpdateGeometry();
            is_window_closed = pointcloud_vis.PollEvents();
            pointcloud_vis.UpdateRender();
        }
    }

    return 0;
}
