#include <aditof/camera.h>
#include <aditof/camera_96tof1_specifics.h>
#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>

#include "../aditof_open3d.h"

static const uint8_t colormap[3 * 256] = {
#include "colormap.txt"
};

using namespace aditof;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    Status status = Status::OK;

    System system;
    status = system.initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize system!";
        return 0;
    }

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
    int camera_rangeMax = cameraDetails.maxDepth;
    int camera_rangeMin = cameraDetails.minDepth;
    int bitCount = cameraDetails.bitCount;

    aditof::IntrinsicParameters intrinsics = cameraDetails.intrinsics;
    double fx = intrinsics.cameraMatrix.at(0);
    double fy = intrinsics.cameraMatrix.at(4);
    double cx = intrinsics.cameraMatrix.at(2);
    double cy = intrinsics.cameraMatrix.at(5);

    /* Enable noise reduction for better results */
    const int smallSignalThreshold = 100;
    auto specifics = camera->getSpecifics();
    auto cam96tof1Specifics =
        std::dynamic_pointer_cast<Camera96Tof1Specifics>(specifics);
    cam96tof1Specifics->setNoiseReductionThreshold(smallSignalThreshold);
    cam96tof1Specifics->enableNoiseReduction(true);

    /* Request frame from camera */
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 0;
    }
    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);
    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    /* Create visualizers for depth and for color image (depth + IR) */
    auto visualized_depth_img = std::make_shared<geometry::Image>();
    visualized_depth_img->Prepare(frameWidth, frameHeight, 1, 2);
    visualization::Visualizer depth_vis;
    depth_vis.CreateVisualizerWindow("Depth Image", frameWidth, frameHeight);
    bool is_geometry_added_depth = false;

    auto visualized_color_img = std::make_shared<geometry::Image>();
    visualized_color_img->Prepare(frameWidth, frameHeight, 3, 1);
    visualization::Visualizer color_vis;
    color_vis.CreateVisualizerWindow("Color Image", frameWidth, frameHeight);
    bool is_geometry_added = false;

    /* Create visualizer for pointcloud */
    visualization::Visualizer pointcloud_vis;
    pointcloud_vis.CreateVisualizerWindow("Pointcloud", 800, 800);
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

        /* Show deth and color images */
        visualized_color_img->data_ = color_image.data_;
        if (!is_geometry_added) {
            color_vis.AddGeometry(visualized_color_img);
            is_geometry_added = true;
        }

        visualized_depth_img->data_ = depth16bits_image.data_;
        if (!is_geometry_added_depth) {
            depth_vis.AddGeometry(visualized_depth_img);
            is_geometry_added_depth = true;
        }
        color_vis.UpdateGeometry();
        color_vis.PollEvents();
        color_vis.UpdateRender();

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
            color_vis.DestroyVisualizerWindow();
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
