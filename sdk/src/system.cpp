#include "system_impl.h"
#include <aditof/camera.h>
#include <aditof/system.h>

namespace aditof {

System::System() : m_impl(new SystemImpl) {}

System::~System() = default;

System::System(System &&) noexcept = default;

System &System::operator=(System &&) noexcept = default;

Status System::initialize() { return m_impl->initialize(); }

Status
System::getCameraList(std::vector<std::shared_ptr<Camera>> &cameraList) const {
    return m_impl->getCameraList(cameraList);
}

Status
System::getCameraListAtIp(std::vector<std::shared_ptr<Camera>> &cameraList,
                          const std::string &ip) const {
    return m_impl->getCameraListAtIp(cameraList, ip);
}

} // namespace aditof
