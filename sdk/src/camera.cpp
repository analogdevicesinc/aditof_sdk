#include "camera.h"
#include "camera_impl.h"
#include "device_interface.h"

namespace aditof {

Camera::Camera(DeviceInterface *device) : m_impl(new CameraImpl(device)) {}

Camera::~Camera() {
    if (m_impl)
        delete m_impl;
};

Camera::Camera(Camera &&other) noexcept {
    this->m_impl = other.m_impl;
    other.m_impl = nullptr;
}

Camera &Camera::operator=(Camera &&other) noexcept {
    std::swap(m_impl, other.m_impl);
    return *this;
}

Status Camera::initialize() { return m_impl->initialize(); }

Status Camera::start() { return m_impl->start(); }

Status Camera::stop() { return m_impl->stop(); }

Status Camera::setMode(const std::string &mode,
                       const std::string &modeFilename) {
    return m_impl->setMode(mode, modeFilename);
}

Status
Camera::getAvailableModes(std::vector<std::string> &availableModes) const {
    return m_impl->getAvailableModes(availableModes);
}

Status Camera::setFrameType(const std::string &frameType) {
    return m_impl->setFrameType(frameType);
}

Status Camera::getAvailableFrameTypes(
    std::vector<std::string> &availableFrameTypes) const {
    return m_impl->getAvailableFrameTypes(availableFrameTypes);
}

Status Camera::requestFrame(Frame *frame, FrameUpdateCallback cb) {
    return m_impl->requestFrame(frame, cb);
}

Status Camera::getDetails(CameraDetails &details) const {
    return m_impl->getDetails(details);
}

DeviceInterface *Camera::getDevice() { return m_impl->getDevice(); }

} // namespace aditof
