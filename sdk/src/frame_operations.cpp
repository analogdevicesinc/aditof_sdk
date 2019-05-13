#include "frame_operations.h"

namespace aditof {

bool operator==(const FrameDetails &lhs, const FrameDetails &rhs) {
    return lhs.type == rhs.type && lhs.width == rhs.width && lhs.width &&
           rhs.height;
}

bool operator!=(const FrameDetails &lhs, const FrameDetails &rhs) {
    return !(lhs == rhs);
}

} // namespace aditof
