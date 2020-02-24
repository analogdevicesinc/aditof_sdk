#include "pointcloud2_msg.h"
using namespace aditof;

PointCloud2Msg::PointCloud2Msg() {}

PointCloud2Msg::PointCloud2Msg(const std::shared_ptr<aditof::Camera> &camera) {
    FrameDataToMsg(camera);
}

void PointCloud2Msg::setMetadataMembers(int width, int height) {
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                  1, sensor_msgs::PointField::FLOAT32, "rgb", 1,
                                  sensor_msgs::PointField::UINT32);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.width = width;
    msg.height = height;

    msg.is_bigendian = false;
    msg.point_step = 4 * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;

    msg.data.resize(width * height * msg.point_step);
}

void PointCloud2Msg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                    uint16_t *frameData) {
    IntrinsicParameters intr = getIntrinsics(camera);

    float fx = intr.cameraMatrix[0];
    float fy = intr.cameraMatrix[4];
    float x0 = intr.cameraMatrix[2];
    float y0 = intr.cameraMatrix[5];

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(msg, "rgb");

    const int frameHeight = static_cast<int>(msg.height);
    const int frameWidth = static_cast<int>(msg.width);

    for (int i = 0; i < frameHeight; i++) {
        for (int j = 0; j < frameWidth;
             j++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
            int index = i * msg.width + j;
            uint16_t depth = frameData[index];

            // rviz expects the data in metres
            float z = depth / 1000.0f;

            *iter_x = z * (j - x0) / fx;
            *iter_y = z * (i - y0) / fy;
            *iter_z = z;

            *iter_rgb = frameData[index];
        }
    }
}

void PointCloud2Msg::FrameDataToMsg(const std::shared_ptr<Camera> &camera) {
    Frame frame;
    uint16_t *frameData = getNewFrame(camera, &frame);

    if (!frameData) {
        LOG(ERROR) << "getNewFrame call failed";
        return;
    }

    FrameDetails fDetails;
    frame.getDetails(fDetails);

    setMetadataMembers(fDetails.width, fDetails.height / 2);

    setDataMembers(camera, frameData);
}

void PointCloud2Msg::publishMsg(const ros::Publisher &pub) { pub.publish(msg); }
