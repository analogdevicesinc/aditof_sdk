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
#ifndef CAMERA_CHICONY_H
#define CAMERA_CHICONY_H

#include <memory>

#include "calibration_chicony_006.h"
#include <aditof/camera.h>
#include <aditof/device_construction_data.h>

class CameraChicony : public aditof::Camera {
  public:
    CameraChicony(std::unique_ptr<aditof::DeviceInterface> device,
                  const aditof::DeviceConstructionData &data);
    ~CameraChicony();

  public: // implements Camera
    aditof::Status initialize();
    aditof::Status start();
    aditof::Status stop();
    aditof::Status setMode(const std::string &mode,
                           const std::string &modeFilename);
    aditof::Status
    getAvailableModes(std::vector<std::string> &availableModes) const;
    aditof::Status setFrameType(const std::string &frameType);
    aditof::Status
    getAvailableFrameTypes(std::vector<std::string> &availableFrameTypes) const;
    aditof::Status requestFrame(aditof::Frame *frame,
                                aditof::FrameUpdateCallback cb);
    aditof::Status getDetails(aditof::CameraDetails &details) const;
    aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const;
    aditof::Status setControl(const std::string &control,
                              const std::string &value);
    aditof::Status getControl(const std::string &control,
                              std::string &value) const;
    std::shared_ptr<aditof::DeviceInterface> getDevice();
    aditof::Status
    getEeproms(std::vector<std::shared_ptr<aditof::EepromInterface>> &eeproms);

  private:
    aditof::Status setNoiseReductionTreshold(uint16_t treshold);
    aditof::Status setIrGammaCorrection(float gamma);

  private:
    aditof::CameraDetails m_details;
    aditof::DeviceConstructionData m_devData;
    std::shared_ptr<aditof::DeviceInterface> m_device;
    std::shared_ptr<aditof::EepromInterface> m_eeprom;
    std::vector<std::string> m_availableControls;
    bool m_devStarted;
    bool m_eepromInitialized;
    CalibrationChicony006 m_calibration;
    uint16_t m_noiseReductionThreshold;
    float m_irGammaCorrection;
};

#endif // CAMERA_CHICONY_H
