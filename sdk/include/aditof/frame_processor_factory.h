#ifndef FRAME_PROCESSOR_FACTORY_H
#define FRAME_PROCESSOR_FACTORY_H

#include <memory>

namespace aditof {

class FrameProcessor;

enum class FrameProcessorType {
    VARIANCE_FILTER,
};

class FrameProcessorFactory {
  public:
    virtual ~FrameProcessorFactory() = default;

  public:
    virtual std::unique_ptr<FrameProcessor>
    createFrameProcessor(FrameProcessorType type) const = 0;
};

} // namespace aditof

#endif // FRAME_PROCESSOR_FACTORY_H
