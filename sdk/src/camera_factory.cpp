#include "camera_factory.h"
#include "camera_96tof1.h"

#include <aditof/device_definitions.h>

namespace aditof {

std::unique_ptr<Camera>
CameraFactory::buildCamera(std::unique_ptr<DeviceInterface> device) {
    using namespace aditof;

    DeviceDetails devDetails;
    device->getDetails(devDetails);
    switch (devDetails.sensorType) {
    case SensorType::SENSOR_96TOF1:
        return std::unique_ptr<Camera>(new Camera96Tof1(std::move(device)));
    }

    return nullptr;
}

} // namespace aditof
