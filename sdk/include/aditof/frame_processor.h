#ifndef FRAME_PROCESSOR_H
#define FRAME_PROCESSOR_H

#include <aditof/status_definitions.h>

namespace aditof {

class Frame;

class FrameProcessor {
  public:
    virtual ~FrameProcessor() = default;

  public:
    virtual Status processFrame(const Frame &inFrame, Frame &outFrame) = 0;
};

} // namespace aditof

#endif // FRAME_PROCESSOR_H
