#ifndef SYSTEM_IMPL_H
#define SYSTEM_IMPL_H

#include <aditof/status_definitions.h>

#include <memory>
#include <vector>

namespace aditof {
class Camera;
}

class DeviceEnumeratorInterface;

class SystemImpl {
  public:
    SystemImpl();
    ~SystemImpl();

  public: // from System
    aditof::Status initialize();
    aditof::Status
    getCameraList(std::vector<aditof::Camera *> &cameraList) const;
    aditof::Status getCameraListAtIp(std::vector<aditof::Camera *> &cameraList,
                                     const std::string &ip) const;

  private:
    std::unique_ptr<DeviceEnumeratorInterface> m_enumerator;
    std::vector<aditof::Camera *> m_cameras;
};

#endif // SYSTEM_IMPL_H
