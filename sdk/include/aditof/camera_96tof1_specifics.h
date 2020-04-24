/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
	
    /**
     * @brief Sets the value of the IR gamma conversion to change the 
	 * IR image contrast.
     * The valid interval is [0, 1], where 0 means maximum contrast 
	 * and 1 minimum contrast
     * @return Status
     */
    Status setIrGammaConversion(float gamma);
	
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
