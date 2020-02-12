#ifndef ADITOF_SENSOR_MSG_H
#define ADITOF_SENSOR_MSG_H

#include <aditof/camera.h>
#include <ros/publisher.h>

class AditofSensorMsg {
  public:
    virtual ~AditofSensorMsg() = default;
    virtual void
    FrameDataToMsg(const std::shared_ptr<aditof::Camera> &camera) = 0;
    virtual void publishMsg(const ros::Publisher &pub) = 0;
};

#endif // ADITOF_SENSOR_MSG_H
