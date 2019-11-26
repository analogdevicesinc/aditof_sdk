#ifndef CAMERA_FACTORY_H
#define CAMERA_FACTORY_H

#include <aditof/camera.h>
#include <aditof/device_interface.h>

#include <memory>

namespace aditof {

class CameraFactory {
  public:
    static std::unique_ptr<Camera>
    buildCamera(std::unique_ptr<DeviceInterface> device);
};

} // namespace aditof

#endif // CAMERA_FACTORY_H
