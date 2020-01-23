#ifndef FRAME_PROCESSOR_FACTORY_H
#define FRAME_PROCESSOR_FACTORY_H

#include <memory>

namespace aditof {

class FrameProcessor;

/**
 * @enum FrameProcessorType
 * @brief Different types of FrameProcessor that are available
 */
enum class FrameProcessorType {
    VARIANCE_FILTER,
};

/**
 * @class FrameProcessorFactory
 * @brief Base class for factories that create FrameProcessor instances
 */
class FrameProcessorFactory {
  public:
    virtual ~FrameProcessorFactory() = default;

  public:
    /**
     * @brief createFrameProcessor
     * @param type - The type of FrameProcessor to create
     * @return std::unique_ptr<FrameProcessor>
     */
    virtual std::unique_ptr<FrameProcessor>
    createFrameProcessor(FrameProcessorType type) const = 0;
};

} // namespace aditof

#endif // FRAME_PROCESSOR_FACTORY_H
