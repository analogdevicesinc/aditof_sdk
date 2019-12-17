#ifndef CAMERA_SPECIFICS_H
#define CAMERA_SPECIFICS_H

#include "sdk_exports.h"
#include "status_definitions.h"

namespace aditof {

/**
 * @class CameraSpecifics
 * @brief Base class for implementations with camera specific API.
 */
class SDK_API CameraSpecifics {
  public:
    /**
     * @brief Destructor
     */
    virtual ~CameraSpecifics() = default;
};

} // namespace aditof

#endif // CAMERA_SPECIFICS_H
