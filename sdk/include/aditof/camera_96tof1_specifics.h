#ifndef CAMERA_96TOF1_SPECIFICS_H
#define CAMERA_96TOF1_SPECIFICS_H

#include "camera.h"
#include "camera_specifics.h"

#include <cstdint>

class Camera96Tof1;

namespace aditof {

/**
 * @enum Revision
 * @brief Enumerates camera revisions
 */
enum class Revision { RevB, RevC };

/**
 * @class Camera96Tof1Specifics
 * @brief Implements the extened API that is specific for the 96 TOF1 camera.
 */
class SDK_API Camera96Tof1Specifics : public CameraSpecifics {
  public:
    /**
     * @brief Constructor
     */
    Camera96Tof1Specifics(Camera *camera);

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

    /**
     * @brief Sets the camera revision type.
     * @return Status
     */
    Status setCameraRevision(Revision revision);

    /**
     * @brief Returns the current camera revision type.
     * @return Revision
     */
    Revision getRevision() const;

  private:
    Status setTresholdAndEnable(uint16_t treshold, bool en);

  private:
    Camera96Tof1 *m_camera;
    bool m_noiseReductionOn;
    uint16_t m_noiseReductionThreshold;
    Revision m_revision;
};

} // namespace aditof

#endif // CAMERA_96TOF1_SPECIFICS_H
