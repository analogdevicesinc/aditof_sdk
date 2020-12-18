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
#include "calibration_fxtof1.h"
#include "aditof/storage_interface.h"
#include "basecode.h"

#include <algorithm>
#include <glog/logging.h>
#include <math.h>

#define MODE_CFG_SIZE 256
static const uint16_t ROMADDR_CFG_BASE[] = {0x100, 0x200};
#define PULSE_CNT_OFFSET 0
#define DEPTH_PWR_OFFSET 2
#define DEPTH_X0_OFFSET 50
#define DEPTH_OFFSET_OFFSET 52
#define DEPTH_3_OFFSET 252
#define DEPTH_2_OFFSET 254

#define COMMON_SIZE 255
#define ROMADDR_COMMOM_BASE 0x300
#define COMMON_BASE_OFFSET 227

static const uint16_t MODE_REG_BASE_ADDR[] = {0x6B80, 0x6BC0};
#define R_MODE_PSPACE 1
#define R_PULSECNT 5
#define R_PULSEHD 6

static const uint16_t MODE_LCORR_BASE_ADDR[] = {0x6A80, 0x6AC0};
#define R_DEPTH_2 2
#define R_DEPTH_3 3
#define R_DEPTH_OFST 4

#define DEPTH_PWR_CNT 48
#define DEPTH_OFST_CNT 49

#define ARRAY_SIZE(X) sizeof(X) / sizeof(X[0])

CalibrationFxTof1::CalibrationFxTof1()
    : m_depth_cache(nullptr), m_geometry_cache(nullptr), m_range(16000),
      m_cal_valid(false) {
    m_afe_code.insert(m_afe_code.end(), &basecode[0],
                      &basecode[ARRAY_SIZE(basecode)]);
}

CalibrationFxTof1::~CalibrationFxTof1() {
    if (m_depth_cache) {
        delete[] m_depth_cache;
    }

    if (m_geometry_cache) {
        delete[] m_geometry_cache;
    }
}

//! ReadCalMap - Read the entire calibration map
/*!
ReadCalMap - Read the entire calibration map from a binary file
\device - Pointer to a device instance
*/
aditof::Status CalibrationFxTof1::readCalMap(
    std::shared_ptr<aditof::StorageInterface> eeprom) {
    using namespace aditof;

    Status status = Status::OK;
    uint8_t mode_data[MODE_CFG_SIZE];
    float intrinsic_data[COMMON_SIZE / sizeof(float)];

    /*Read the mode data*/
    for (int i = 0; i < 2; i++) {
        eeprom->read(ROMADDR_CFG_BASE[i], mode_data, MODE_CFG_SIZE);

        if (mode_data[0] == 0xFF) {
            LOG(WARNING)
                << "Invalid calibration in EEPROM, using default settings";
            return Status::OK;
        }

        m_mode_settings[i].pulse_cnt =
            (uint16_t)mode_data[PULSE_CNT_OFFSET] |
            ((uint16_t)mode_data[PULSE_CNT_OFFSET + 1] << 8);

        memcpy(m_mode_settings[i].depth_pwr, &mode_data[DEPTH_PWR_OFFSET],
               DEPTH_PWR_CNT);

        m_mode_settings[i].depth_x0 =
            (uint16_t)mode_data[DEPTH_X0_OFFSET] |
            ((uint16_t)mode_data[DEPTH_X0_OFFSET + 1] << 8);

        for (int j = DEPTH_OFFSET_OFFSET;
             j < DEPTH_OFFSET_OFFSET + DEPTH_OFST_CNT; j += 2) {
            m_mode_settings[i].depth_offset[j - DEPTH_OFFSET_OFFSET] =
                (uint16_t)mode_data[j] | ((uint16_t)mode_data[j + 1] << 8);
        }

        m_mode_settings[i].depth3 =
            (uint16_t)mode_data[DEPTH_3_OFFSET] |
            ((uint16_t)mode_data[DEPTH_3_OFFSET + 1] << 8);

        m_mode_settings[i].depth2 =
            (uint16_t)mode_data[DEPTH_2_OFFSET] |
            ((uint16_t)mode_data[DEPTH_2_OFFSET + 1] << 8);
    }

    /*Replace the settings in the base code with the eeprom values*/
    for (int i = 0; i < 2; i++) {
        auto it = std::find(m_afe_code.begin(), m_afe_code.end(),
                            MODE_REG_BASE_ADDR[i] + R_MODE_PSPACE);
        if (it == m_afe_code.end()) {
            LOG(WARNING) << "Could not find R_MODE_PSPACE";
            return Status::INVALID_ARGUMENT;
        }

        uint16_t pulse_space = *(it++);
        uint16_t pulse_cnt = m_mode_settings[i].pulse_cnt;
        uint16_t pulse_hd =
            (((pulse_cnt - 1) * pulse_space + 90) / 928 + 3) * 36 + 57;
        it = std::find(m_afe_code.begin(), m_afe_code.end(),
                       MODE_REG_BASE_ADDR[i] + R_PULSECNT);
        if (it == m_afe_code.end()) {
            LOG(WARNING) << "Could not find R_PULSECNT";
            return Status::INVALID_ARGUMENT;
        }
        *(it + 1) = pulse_cnt;

        it = std::find(m_afe_code.begin(), m_afe_code.end(),
                       MODE_REG_BASE_ADDR[i] + R_PULSEHD);
        if (it == m_afe_code.end()) {
            LOG(WARNING) << "Could not find R_PULSEHD";
            return Status::INVALID_ARGUMENT;
        }
        *(it + 1) = pulse_hd;

        it = std::find(m_afe_code.begin(), m_afe_code.end(),
                       MODE_LCORR_BASE_ADDR[i] + R_DEPTH_2);
        if (it == m_afe_code.end()) {
            LOG(WARNING) << "Could not find R_DEPTH_2";
            return Status::INVALID_ARGUMENT;
        }
        *(it + 1) = m_mode_settings[i].depth2;

        it = std::find(m_afe_code.begin(), m_afe_code.end(),
                       MODE_LCORR_BASE_ADDR[i] + R_DEPTH_3);
        if (it == m_afe_code.end()) {
            LOG(WARNING) << "Could not find R_DEPTH_3";
            return Status::INVALID_ARGUMENT;
        }
        *(it + 1) = m_mode_settings[i].depth3;

        it = std::find(m_afe_code.begin(), m_afe_code.end(),
                       MODE_LCORR_BASE_ADDR[i] + R_DEPTH_OFST) +
             1;
        if (it == m_afe_code.end()) {
            LOG(WARNING) << "Could not find R_DEPTH_OFST";
            return Status::INVALID_ARGUMENT;
        }
        for (int j = 0; j < DEPTH_OFST_CNT; j++) {
            *it = m_mode_settings[i].depth_offset[j];
            it += 2;
        }
    }

    /*Read the intrinsics and distortion params*/
    eeprom->read(ROMADDR_COMMOM_BASE + COMMON_BASE_OFFSET,
                 (uint8_t *)intrinsic_data, MODE_CFG_SIZE - COMMON_BASE_OFFSET);
    m_intrinsics.insert(m_intrinsics.end(), &intrinsic_data[0],
                        &intrinsic_data[ARRAY_SIZE(intrinsic_data)]);

    m_cal_valid = true;

    return status;
}

//! getAfeFirmware - Get the firmware for a mode
/*!
getAfeFirmware - Get the firmware for a mode
\param mode - Camera mode
\param data - Buffer where to store the firmware
*/
aditof::Status
CalibrationFxTof1::getAfeFirmware(const std::string &mode,
                                  std::vector<uint16_t> &data) const {
    using namespace aditof;

    data = m_afe_code;

    return Status::OK;
}

//! getIntrinsic - Get the geometric camera calibration
/*!
getIntrinsic - Get the geometric camera clibration
\param key - Specifies which calibration values to get:
             intrinsincs or distortion coefficients
\param data - Buffer to store the returned data
*/
aditof::Status CalibrationFxTof1::getIntrinsic(float key,
                                               std::vector<float> &data) const {
    using namespace aditof;

    bool validParam = (INTRINSIC == key) || (DISTORTION_COEFFICIENTS == key);

    if (!validParam || !m_cal_valid) {
        LOG(WARNING) << "Invalid intrinsic " << std::to_string(key).c_str();
        return Status::INVALID_ARGUMENT;
    }

    if (key == INTRINSIC) {
        data.insert(data.end(), m_intrinsics.begin(), m_intrinsics.end() - 3);
    } else {
        data.insert(data.end(), m_intrinsics.end() - 3, m_intrinsics.end());
    }

    return Status::OK;
}

//! setMode - Sets the mode to be used for depth calibration
/*!
setMode - Sets the mode to be used for depth calibration
\param mode - Camera depth mode
\param range - Max range for selected mode
\param frameWidth - Width of the depth image in pixels
\param frameHeight - Height of the depth image in pixels
*/
aditof::Status CalibrationFxTof1::setMode(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    const std::string &mode, int range, unsigned int frameWidth,
    unsigned int frameheight) {
    using namespace aditof;

    Status status = Status::OK;
    std::vector<float> cameraMatrix;
    uint16_t mode_id = (mode == "near" ? 0 : 1);

    m_range = range;

    status = getIntrinsic(INTRINSIC, cameraMatrix);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read intrinsic from eeprom";
    } else {
        LOG(INFO) << "Camera intrinsic parameters:\n"
                  << "    fx: " << cameraMatrix[2] << "\n"
                  << "    fy: " << cameraMatrix[3] << "\n"
                  << "    cx: " << cameraMatrix[0] << "\n"
                  << "    cy: " << cameraMatrix[1];
        buildGeometryCalibrationCache(cameraMatrix, frameWidth, frameheight);
    }

    /*Execute the mode change command*/
    uint16_t afeRegsAddr[] = {0x4000, 0x4001, 0x7c22};
    uint16_t afeRegsVal[] = {mode_id, 0x0004, 0x0004};
    depthSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);

    return Status::OK;
}

//! calibrateCameraGeometry - Compensate for lens distorsion in the depth data
/*!
calibrateCameraGeometry - Compensate for lens distorsion in the depth data
\param frame - Buffer with the depth data, used to return the calibrated data
\param frame_size - Number of samples in the frame data
*/
aditof::Status CalibrationFxTof1::calibrateCameraGeometry(uint16_t *frame,
                                                          uint32_t frame_size) {
    using namespace aditof;

    if (!m_cal_valid) {
        return Status::OK;
    }

    for (uint32_t i = 0; i < frame_size; i++) {
        if (frame[i] != m_range) {
            frame[i] = static_cast<uint16_t>(frame[i] * m_geometry_cache[i]);
        }
        if (frame[i] > m_range) {
            frame[i] = m_range;
        }
    }

    return Status::OK;
}

// Create a cache to speed up depth geometric camera calibration computation
void CalibrationFxTof1::buildGeometryCalibrationCache(
    const std::vector<float> &cameraMatrix, unsigned int width,
    unsigned int height) {

    float fx = cameraMatrix[0];
    float fy = cameraMatrix[4];
    float x0 = cameraMatrix[2];
    float y0 = cameraMatrix[5];

    if (m_geometry_cache) {
        delete[] m_geometry_cache;
    }

    m_geometry_cache = new double[width * height];
    for (uint16_t i = 0; i < height; i++) {
        for (uint16_t j = 0; j < width; j++) {

            double tanXAngle = (x0 - j) / fx;
            double tanYAngle = (y0 - i) / fy;

            m_geometry_cache[i * width + j] =
                1.0 / sqrt(1 + tanXAngle * tanXAngle + tanYAngle * tanYAngle);
        }
    }
}
