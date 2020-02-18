#ifndef MESSAGE_FACTORY_H
#define MESSAGE_FACTORY_H

#include "pointcloud2_msg.h"

enum class MessageType {
    sensor_msgs_PointCloud2
    // TODO: add support for other types of messages
    //,sensor_msgs_Image, sensor_msgs_CameraInfo
};

class MessageFactory {
  public:
    static AditofSensorMsg *
    create(const std::shared_ptr<aditof::Camera> &camera, MessageType type);
};

#endif // MESSAGE_FACTORY_H
