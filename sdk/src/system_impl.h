#ifndef SYSTEM_IMPL_H
#define SYSTEM_IMPL_H

#include <aditof/status_definitions.h>

#include <memory>
#include <vector>

namespace aditof {
class Camera;
class DeviceEnumeratorInterface;
} // namespace aditof

class SystemImpl {
  public:
    SystemImpl();
    ~SystemImpl();

  public: // from System
    aditof::Status initialize();
    aditof::Status getCameraList(
        std::vector<std::shared_ptr<aditof::Camera>> &cameraList) const;
    aditof::Status
    getCameraListAtIp(std::vector<std::shared_ptr<aditof::Camera>> &cameraList,
                      const std::string &ip) const;

  private:
    std::unique_ptr<aditof::DeviceEnumeratorInterface> m_enumerator;
    std::vector<std::shared_ptr<aditof::Camera>> m_cameras;
};

#endif // SYSTEM_IMPL_H
