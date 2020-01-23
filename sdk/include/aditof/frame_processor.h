#ifndef FRAME_PROCESSOR_H
#define FRAME_PROCESSOR_H

#include <aditof/status_definitions.h>

namespace aditof {

class Frame;

/**
 * @class FrameProcessor
 * @brief The base class for implementations for FrameProcessor
 */
class FrameProcessor {
  public:
    /**
     * @brief Destructor
     */
    virtual ~FrameProcessor() = default;

  public:
    /**
     * @brief processFrame
     * @param inFrame - The frame which is used as input data
     * @param[out] outFrame - The frame which gets modified during the process
     * @return Status
     */
    virtual Status processFrame(const Frame &inFrame, Frame &outFrame) = 0;
};

} // namespace aditof

#endif // FRAME_PROCESSOR_H
