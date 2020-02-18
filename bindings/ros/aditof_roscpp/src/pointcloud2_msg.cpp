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
    msg.is_dense = true;

    msg.data.resize(width * height * msg.point_step);
}

void PointCloud2Msg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                    uint16_t *frameData) {
    IntrinsicParameters intr = getIntrinsics(camera);

    float fx = intr.cameraMatrix[0] * intr.pixelWidth;
    float fy = intr.cameraMatrix[4] * intr.pixelHeight;
    float x0 = intr.cameraMatrix[2] * intr.pixelWidth;
    float y0 = intr.cameraMatrix[5] * intr.pixelHeight;

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(msg, "rgb");
    float u, v, z;
    float x_w, y_w, z_w;
    float x_c, y_c, z_c;
    float x_n, y_n, z_n;

    float near = getRangeMin(camera);
    float far = getRangeMax(camera);

    float left = (-1.0) * near * (0.0 - x0) / fx;
    float bottom = (-1.0) * near * (0.0 - y0) / fy;

    float right = (-1.0) * near * (intr.pixelWidth * (msg.width - 1) - x0) / fx;
    float top = (-1.0) * near * (intr.pixelHeight * (msg.height - 1) - y0) / fy;

    const int frameHeight = static_cast<int>(msg.height);
    const int frameWidth = static_cast<int>(msg.width);

    for (int i = 0; i < frameHeight; i++) {
        for (int j = 0; j < frameWidth;
             j++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
            u = j * intr.pixelWidth;
            v = i * intr.pixelHeight;
            z = frameData[i * msg.width + j];

            // world space
            z_w = z;
            x_w = z_w * (u - x0) / fx;
            y_w = z_w * (v - y0) / fy;

            // clip space
            x_c = (2 * near * x_w + z_w * (right + left)) / (right - left);
            y_c = (2 * near * y_w + z_w * (top + bottom)) / (top - bottom);
            z_c = ((-1.0) * (far + near) * z_w - 2 * far * near) / (far - near);

            // normalized device space
            x_n = (-1.0) * x_c / z_w;
            y_n = (-1.0) * y_c / z_w;
            z_n = (-1.0) * z_c / z_w;

            *iter_x = x_n;
            *iter_y = y_n;
            *iter_z = z_n;

            *iter_rgb = frameData[i * msg.width + j];
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
