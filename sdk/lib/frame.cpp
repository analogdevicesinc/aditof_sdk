#include "frame_impl.h"
#include <aditof/frame.h>

namespace aditof {

Frame::Frame() : m_impl(new FrameImpl) {}

Frame::~Frame() {
    if (m_impl)
        delete m_impl;
}

Frame::Frame(Frame &&other) noexcept {
    this->m_impl = other.m_impl;
    other.m_impl = nullptr;
};

Frame &Frame::operator=(Frame &&other) noexcept {
    std::swap(m_impl, other.m_impl);
    return *this;
}

Frame::Frame(const Frame &op) : m_impl(new FrameImpl(*op.m_impl)) {}

Frame &Frame::operator=(const Frame &op) {
    if (this != &op) {
        delete m_impl;
        m_impl = (new FrameImpl(*op.m_impl));
    }

    return *this;
}

Status Frame::setDetails(const FrameDetails &details) {
    return m_impl->setDetails(details);
}

Status Frame::getDetails(FrameDetails &details) const {
    return m_impl->getDetails(details);
}

Status Frame::getData(FrameDataType dataType, uint16_t **dataPtr) {
    return m_impl->getData(dataType, dataPtr);
}

} // namespace aditof
