#ifndef CAMERA_FACTORY_H
#define CAMERA_FACTORY_H

#include <aditof/camera.h>
#include <aditof/device_interface.h>

namespace aditof {

class CameraFactory {
  public:
    static Camera *buildCamera(DeviceInterface *device);
};

} // namespace aditof

#endif // CAMERA_FACTORY_H
