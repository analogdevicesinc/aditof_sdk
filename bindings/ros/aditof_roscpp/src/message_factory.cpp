#include "message_factory.h"

AditofSensorMsg *
MessageFactory::create(const std::shared_ptr<aditof::Camera> &camera,
                       MessageType type) {
    switch (type) {
    case MessageType::sensor_msgs_PointCloud2:
        return new PointCloud2Msg(camera);
    }
    return nullptr;
}
