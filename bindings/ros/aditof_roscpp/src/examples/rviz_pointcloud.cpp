#include "message_factory.h"
#include <aditof_utils.h>
#include <ros/ros.h>

using namespace aditof;

int main(int argc, char **argv) {

    std::shared_ptr<Camera> camera = initCameraEthernet(argc, argv);
    if (camera.get() == nullptr) {
        ROS_ERROR("initCamera call failed");
        return 0;
    }

    ros::init(argc, argv, "aditof_rviz_node");

    ros::NodeHandle nHandle;
    ros::Publisher frame_pubisher =
        nHandle.advertise<sensor_msgs::PointCloud2>("aditof_pcloud", 100);

    applyNoiseReduction(camera, argc, argv);

    AditofSensorMsg *pclMsg =
        MessageFactory::create(camera, MessageType::sensor_msgs_PointCloud2);

    if (!pclMsg) {
        ROS_ERROR("pointcloud message creation failed");
    }

    while (ros::ok()) {
        dynamic_cast<PointCloud2Msg *>(pclMsg)->FrameDataToMsg(camera);
        pclMsg->publishMsg(frame_pubisher);
    }

    delete pclMsg;

    return 0;
}
