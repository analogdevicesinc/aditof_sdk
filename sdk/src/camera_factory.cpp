#include "camera_factory.h"
#include "camera_96tof1.h"

#include <aditof/device_definitions.h>

namespace aditof {

Camera *CameraFactory::buildCamera(DeviceInterface *device) {
    using namespace aditof;

    DeviceDetails devDetails;
    device->getDetails(devDetails);
    switch (devDetails.sensorType) {
    case SensorType::SENSOR_96TOF1:
        return new Camera96Tof1(device);
    }

    return nullptr;
}

} // namespace aditof
