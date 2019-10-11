#ifndef VARIANCE_FILTER_H
#define VARIANCE_FILTER_H

#include <aditof/frame_processor.h>

namespace aditof {

class VarianceFilter : public FrameProcessor {
  public:
    Status processFrame(const Frame &inFrame, Frame &outFrame) override;
};

} // namespace aditof

#endif // VARIANCE_FILTER_H
