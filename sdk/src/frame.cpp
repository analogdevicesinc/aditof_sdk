#include "frame_impl.h"
#include <aditof/frame.h>

namespace aditof {

Frame::Frame() : m_impl(new FrameImpl) {}

Frame::~Frame() = default;

Frame::Frame(Frame &&) noexcept = default;

Frame &Frame::operator=(Frame &&) noexcept = default;

Frame::Frame(const Frame &op) : m_impl(new FrameImpl(*op.m_impl)) {}

Frame &Frame::operator=(const Frame &op) {
    if (this != &op) {
        m_impl.reset(new FrameImpl(*op.m_impl));
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
