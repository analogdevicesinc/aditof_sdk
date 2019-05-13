#ifndef FRAME_OPERATIONS_H
#define FRAME_OPERATIONS_H

#include "frame_definitions.h"

namespace aditof {

/**
 * @brief operator ==
 *
 * @param lhs - LHS FrameDetails
 * @param rhs - RHS FrameDetails
 * @return bool
 */
bool operator==(const FrameDetails &lhs, const FrameDetails &rhs);

/**
 * @brief operator !=
 *
 * @param lhs - LHS FrameDetails
 * @param rhs - RHS FrameDetails
 * @return bool
 */
bool operator!=(const FrameDetails &lhs, const FrameDetails &rhs);

}; // namespace aditof

#endif // FRAME_OPERATIONS_H
