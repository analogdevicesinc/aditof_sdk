#include <aditof/filters_factory.h>
#include <aditof/variance_filter.h>

using namespace aditof;

std::unique_ptr<FrameProcessor>
FiltersFactory::createFrameProcessor(FrameProcessorType type) const {
    switch (type) {

    case FrameProcessorType::VARIANCE_FILTER:
        return std::unique_ptr<FrameProcessor>(new VarianceFilter());
    }

    return nullptr;
}
