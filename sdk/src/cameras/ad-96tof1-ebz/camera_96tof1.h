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
#ifndef CAMERA_96TOF1_H
#define CAMERA_96TOF1_H

#include "calibration_96tof1.h"

#include <memory>

#include "aditof/depth_sensor_interface.h"
#include "aditof/storage_interface.h"
#include "aditof/temperature_sensor_interface.h"
#include <aditof/camera.h>

class Camera96Tof1 : public aditof::Camera {
  public:
    Camera96Tof1(
        std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
        std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &tSensors);
    ~Camera96Tof1();

  public: // implements Camera
    aditof::Status initialize() override;
    aditof::Status start() override;
    aditof::Status stop() override;
    aditof::Status setMode(const std::string &mode,
                           const std::string &modeFilename) override;
    aditof::Status
    getAvailableModes(std::vector<std::string> &availableModes) const override;
    aditof::Status setFrameType(const std::string &frameType) override;
    aditof::Status getAvailableFrameTypes(
        std::vector<std::string> &availableFrameTypes) const override;
    aditof::Status requestFrame(aditof::Frame *frame,
                                aditof::FrameUpdateCallback cb) override;
    aditof::Status getDetails(aditof::CameraDetails &details) const override;
    aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const override;
    aditof::Status setControl(const std::string &control,
                              const std::string &value) override;
    aditof::Status getControl(const std::string &control,
                              std::string &value) const override;
    std::shared_ptr<aditof::DepthSensorInterface> getSensor() override;
    aditof::Status
    getEeproms(std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms)
        override;
    aditof::Status getTemperatureSensors(
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &sensors) override;

  private:
    aditof::Status setNoiseReductionTreshold(uint16_t treshold);
    aditof::Status setIrGammaCorrection(float gamma);

  private:
    aditof::CameraDetails m_details;
    std::shared_ptr<aditof::DepthSensorInterface> m_depthSensor;
    std::shared_ptr<aditof::StorageInterface> m_eeprom;
    std::shared_ptr<aditof::TemperatureSensorInterface> m_afeTempSensor;
    std::shared_ptr<aditof::TemperatureSensorInterface> m_laserTempSensor;
    bool m_devStarted;
    bool m_eepromInitialized;
    bool m_tempSensorsInitialized;
    std::vector<std::string> m_availableControls;
    Calibration96Tof1 m_calibration;
    uint16_t m_noiseReductionThreshold;
    float m_irGammaCorrection;
    std::string m_revision;
};

#endif // CAMERA_96TOF1_H
