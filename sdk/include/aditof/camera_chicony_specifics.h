#ifndef CAMERA_CHICONY_SPECIFICS_H
#define CAMERA_CHICONY_SPECIFICS_H

#include "camera.h"
#include "camera_specifics.h"

#include <cstdint>

class CameraChicony;

namespace aditof {

/**
 * @class CameraChiconySpecifics
 * @brief Implements the extened API that is specific for the Chicony camera.
 */
class SDK_API CameraChiconySpecifics : public CameraSpecifics {
  public:
    /**
     * @brief Constructor
     */
    CameraChiconySpecifics(Camera *camera);

    /**
     * @brief Enables the noise reduction feature.
     * @return Status
     */
    Status enableNoiseReduction(bool en);

    /**
     * @brief Returns the last state that has been set for the noise reduction
     * feature.
     * @return bool
     */
    bool noiseReductionEnabled() const;

    /**
     * @brief Sets the value of the threshold of the noise reduction feature.
     * The valid interval is [0, 16383].
     * @return Status
     */
    Status setNoiseReductionThreshold(uint16_t threshold);

    /**
     * @brief Returns the last value that has been set for the threshold of the
     * noise reduction.
     * @return uint16_t
     */
    uint16_t noiseReductionThreshold() const;

  private:
    Status setTresholdAndEnable(uint16_t treshold, bool en);

  private:
    CameraChicony *m_camera;
    bool m_noiseReductionOn;
    uint16_t m_noiseReductionThreshold;
};

} // namespace aditof

#endif // CAMERA_CHICONY_SPECIFICS_H
