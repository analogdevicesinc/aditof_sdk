#ifndef POINTCLOUD2_MSG_H
#define POINTCLOUD2_MSG_H

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <glog/logging.h>

#include "aditof_sensor_msg.h"
#include "aditof_utils.h"
#include <sensor_msgs/point_cloud2_iterator.h>

class PointCloud2Msg : public AditofSensorMsg {
  public:
    PointCloud2Msg();
    PointCloud2Msg(const std::shared_ptr<aditof::Camera> &camera);

    /**
     * @brief Each message corresponds to one frame
     */
    sensor_msgs::PointCloud2 msg;

    /**
     * @brief Assigns values to the message fields concerning metadata
     */
    void setMetadataMembers(int width, int height);

    /**
     * @brief Assigns values to the message fields concerning the point data
     */
    void setDataMembers(const std::shared_ptr<aditof::Camera> &camera,
                        uint16_t *frameData);
    /**
     * @brief Converts the frame data to a message
     */
    void FrameDataToMsg(const std::shared_ptr<aditof::Camera> &camera);

    /**
     * @brief Publishes a message
     */
    void publishMsg(const ros::Publisher &pub);
};

#endif // POINTCLOUD2_MSG_H
