#ifndef FRAME_H
#define FRAME_H

#include "frame_definitions.h"
#include "sdk_exports.h"
#include "status_definitions.h"

#include <memory>

class FrameImpl;

namespace aditof {

/**
 * @class Frame
 * @brief Frame of a camera.
 */
class SDK_API Frame {
  public:
    /**
     * @brief Constructor
     */
    Frame();

    /**
     * @brief Destructor
     */
    ~Frame();

    /**
     * @brief Copy constructor
     */
    Frame(const Frame &op);

    /**
     * @brief Copy assignment
     */
    Frame &operator=(const Frame &op);

    /**
     * @brief Move constructor
     */
    Frame(Frame &&) noexcept;
    /**
     * @brief Move assignment
     */
    Frame &operator=(Frame &&) noexcept;

  public:
    /**
     * @brief Configures the frame with the given details
     * @param details
     * @return Status
     */
    Status setDetails(const FrameDetails &details);

    /**
     * @brief Gets the current details of the frame
     * @param[out] details
     * @return Status
     */
    Status getDetails(FrameDetails &details) const;

    /**
     * @brief Gets the address where the specified data is being stored
     * @param dataType
     * @param[out] dataPtr
     * @return Status
     */
    Status getData(FrameDataType dataType, uint16_t **dataPtr);

  private:
    std::unique_ptr<FrameImpl> m_impl;
};

} // namespace aditof

#endif // FRAME_H
