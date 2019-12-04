#ifndef FILTERS_FACTORY_H
#define FILTERS_FACTORY_H

#include <aditof/frame_processor_factory.h>

namespace aditof {

class FiltersFactory : public FrameProcessorFactory {
  public:
    virtual std::unique_ptr<FrameProcessor>
    createFrameProcessor(FrameProcessorType type) const override;
};

} // namespace aditof

#endif // FILTERS_FACTORY_H
