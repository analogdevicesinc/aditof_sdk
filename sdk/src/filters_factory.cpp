#include <aditof/filters_factory.h>
#include <aditof/variance_filter.h>

using namespace aditof;

FrameProcessor *
FiltersFactory::createFrameProcessor(FrameProcessorType type) const {
    switch (type) {

    case FrameProcessorType::VARIANCE_FILTER:
        return new VarianceFilter();
    }

    return nullptr;
}
