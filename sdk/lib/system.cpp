#include "system_impl.h"
#include <aditof/camera.h>
#include <aditof/system.h>

namespace aditof {

System::System() : m_impl(new SystemImpl) {}

System::~System() {
    if (m_impl)
        delete m_impl;
}

System::System(System &&other) noexcept {
    this->m_impl = other.m_impl;
    other.m_impl = nullptr;
};

System &System::operator=(System &&other) noexcept {
    std::swap(m_impl, other.m_impl);
    return *this;
}

Status System::initialize() { return m_impl->initialize(); }

Status System::getCameraList(std::vector<Camera *> &cameraList) const {
    return m_impl->getCameraList(cameraList);
}

} // namespace aditof
