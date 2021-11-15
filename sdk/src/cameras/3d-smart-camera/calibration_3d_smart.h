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
#ifndef CALIBRATION_3D_SMART_H
#define CALIBRATION_3D_SMART_H

#include "aditof/depth_sensor_interface.h"
#include "aditof/storage_interface.h"
#include "aditof/temperature_sensor_interface.h"
#include <aditof/status_definitions.h>
#include <iostream>
#include <memory>
#include <stdint.h>
#include <vector>

#define INTRINSIC 5
#define DISTORTION_COEFFICIENTS 6
#ifndef EEPROM_SERIAL_ADDR
#define EEPROM_SERIAL_ADDR 0x00010014
#define EEPROM_SERIAL_LENGHT 12
#endif

namespace aditof {
class StorageInterface;
}

class Calibration3D_Smart {
  public:
    Calibration3D_Smart();
    ~Calibration3D_Smart();

  public:
    aditof::Status readCalMap(std::shared_ptr<aditof::StorageInterface> eeprom);
    aditof::Status getAfeFirmware(const std::string &mode,
                                  std::vector<uint16_t> &data) const;
    aditof::Status getIntrinsic(float key, std::vector<float> &data) const;
    aditof::Status setMode(std::shared_ptr<aditof::DepthSensorInterface> sensor,
                           const std::string &mode, int range,
                           unsigned int frameWidth, unsigned int frameheight);
    aditof::Status calibrateDepth(uint16_t *frame, uint32_t frame_size);
    aditof::Status calibrateCameraGeometry(uint16_t *frame,
                                           uint32_t frame_size);
    aditof::Status distortionCorrection(uint16_t *frame, unsigned int width,
                                        unsigned int height);

  private:
    void buildDepthCalibrationCache(float gain, float offset,
                                    int16_t maxPixelValue, int range);
    void buildGeometryCalibrationCache(const std::vector<float> &cameraMatrix,
                                       unsigned int width, unsigned int height);
    void buildDistortionCorrectionCache(unsigned int width,
                                        unsigned int height);

  private:
    //! mode_struct - Structure to hold the packet consisting of map of parameters
    /*!
      mode_struct provides structure to hold the packet(sub map) of parameters
  */
    struct mode_struct {
        uint16_t pulse_cnt;
        uint8_t depth_pwr[48];
        uint16_t depth_x0;
        uint16_t depth_offset[49];
        uint16_t depth3;
        uint16_t depth2;
    };

  private:
    uint16_t *m_depth_cache;
    double *m_geometry_cache;
    double *m_distortion_cache;
    int m_range;
    mode_struct m_mode_settings[2];
    std::vector<float> m_intrinsics;
    std::vector<uint16_t> m_afe_code;
    bool m_cal_valid;
};

#endif /*CALIBRATION_3D_SMART_H*/
