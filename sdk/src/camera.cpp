#include "camera.h"
#include "camera_impl.h"

namespace aditof {

Camera::Camera(std::unique_ptr<CameraImpl> impl) : m_impl(std::move(impl)) {}

Camera::~Camera() = default;

Camera::Camera(Camera &&) noexcept = default;

Camera &Camera::operator=(Camera &&) noexcept = default;

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

Status Camera::requestFrame(std::shared_ptr<Frame> frame,
                            FrameUpdateCallback cb) {
    return m_impl->requestFrame(frame, cb);
}

Status Camera::getDetails(CameraDetails &details) const {
    return m_impl->getDetails(details);
}

std::shared_ptr<DeviceInterface> Camera::getDevice() {
    return m_impl->getDevice();
}

} // namespace aditof
