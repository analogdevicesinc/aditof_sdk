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
#include "calibration_3d_smart.h"
#include "aditof/storage_interface.h"
#include "basecode.h"

#include <algorithm>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#include <cstring>
#endif
#include <math.h>

#define MODE_CFG_SIZE 256
static const uint32_t ROMADDR_CFG_BASE[] = {0x00010200, 0x00010100};
#define PULSE_CNT_OFFSET 0
#define DEPTH_PWR_OFFSET 2
#define DEPTH_X0_OFFSET 50
#define DEPTH_OFFSET_OFFSET 52
#define DEPTH_3_OFFSET 252
#define DEPTH_2_OFFSET 254

#define COMMON_SIZE 255
#define ROMADDR_COMMOM_BASE 0x000103E4
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

Calibration3D_Smart::Calibration3D_Smart()
    : m_depth_cache(nullptr), m_geometry_cache(nullptr),
      m_distortion_cache(nullptr), m_range(16000), m_cal_valid(false) {
    m_afe_code.insert(m_afe_code.end(), &basecode[0],
                      &basecode[ARRAY_SIZE(basecode)]);
}

Calibration3D_Smart::~Calibration3D_Smart() {
    if (m_depth_cache) {
        delete[] m_depth_cache;
    }

    if (m_geometry_cache) {
        delete[] m_geometry_cache;
    }

    if (m_distortion_cache) {
        delete[] m_distortion_cache;
    }

#if defined(CUDA_ON_TARGET)
    cudaObj.freeAll();
#endif
}

//! ReadCalMap - Read the entire calibration map
/*!
ReadCalMap - Read the entire calibration map from a binary file
\device - Pointer to a device instance
*/
aditof::Status Calibration3D_Smart::readCalMap(
    std::shared_ptr<aditof::StorageInterface> eeprom) {
    using namespace aditof;

    Status status = Status::OK;
    uint8_t mode_data[MODE_CFG_SIZE];
    float intrinsic_data[(COMMON_SIZE + 1) / sizeof(float)];

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
             j < DEPTH_OFFSET_OFFSET + DEPTH_OFST_CNT * 2; j += 2) {
            m_mode_settings[i].depth_offset[(j - DEPTH_OFFSET_OFFSET) / 2] =
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
        uint16_t pulse_space = *(it + 1);
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

    uint8_t intrinsics_byte[28];
    uint32_t intrinsics_full[7];

    eeprom->read(ROMADDR_COMMOM_BASE, intrinsics_byte, 28);

    for (int i = 0; i < 28; i = i + 4) {
        intrinsics_full[i / 4] =
            intrinsics_byte[i] | (intrinsics_byte[i + 1] << 8) |
            (intrinsics_byte[i + 2] << 16) | (intrinsics_byte[i + 3] << 24);
    }

    memcpy(intrinsic_data, &intrinsics_full, 28);
    m_intrinsics.insert(m_intrinsics.end(), &intrinsic_data[0],
                        &intrinsic_data[7]);
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
Calibration3D_Smart::getAfeFirmware(const std::string &mode,
                                    std::vector<uint16_t> &data) const {
    using namespace aditof;

    data = m_afe_code;

    return Status::OK;
}

//! getIntrinsic - Get the geometric camera calibration
/*!
getIntrinsic - Get the geometric camera calibration
\param key - Specifies which calibration values to get:
             INTRINSIC for [fx, 0, cx, 0, fy, cy, 0, 0, 1]
             DISTORTION_COEFFICIENTS for [k1, k2, p1, p2, k3]
\param data - Buffer to store the returned data
*/
aditof::Status
Calibration3D_Smart::getIntrinsic(float key, std::vector<float> &data) const {
    using namespace aditof;

    bool validParam = (INTRINSIC == key) || (DISTORTION_COEFFICIENTS == key);

    if (!validParam || !m_cal_valid) {
        LOG(WARNING) << "Invalid intrinsic " << std::to_string(key).c_str();
        return Status::INVALID_ARGUMENT;
    }

    if (key == INTRINSIC) {
        std::vector<float> intrinsic{m_intrinsics[2],
                                     0,
                                     m_intrinsics[0],
                                     0,
                                     m_intrinsics[3],
                                     m_intrinsics[1],
                                     0,
                                     0,
                                     1};
        data.insert(data.end(), intrinsic.begin(), intrinsic.end());
    } else {
        std::vector<float> dist_coeff{m_intrinsics[4], m_intrinsics[5], 0, 0,
                                      m_intrinsics[6]};
        data.insert(data.end(), dist_coeff.begin(), dist_coeff.end());
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
aditof::Status Calibration3D_Smart::setMode(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    const std::string &mode, int range, unsigned int frameWidth,
    unsigned int frameHeight) {
    using namespace aditof;

    Status status = Status::OK;
    std::vector<float> cameraMatrix;
    std::vector<float> distortionCoeffs;
    uint16_t mode_id = (mode == "near" ? 0 : 1);
    const int16_t pixelMaxValue = (1 << 12) - 1; // 4095
    float gain = (mode == "near" ? 0.5f : 1.15f);
    float offset = 0.0;

#if defined(CUDA_ON_TARGET)

    m_cudaParameters[0] = (double)frameWidth;
    m_cudaParameters[1] = (double)frameHeight;
    m_cudaParameters[11] = (double)gain;
    m_cudaParameters[12] = (double)offset;
    m_cudaParameters[13] = (double)pixelMaxValue;
    m_cudaParameters[14] = (double)range;
    cudaObj.setParameters(m_cudaParameters);
    cudaObj.buildDepthCorrectionCache();
#else
    buildDepthCalibrationCache(gain, offset, pixelMaxValue, range);

#endif

    m_range = range;

    status = getIntrinsic(INTRINSIC, cameraMatrix);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read intrinsic from eeprom";
    } else {
        LOG(INFO) << "Camera intrinsic parameters:\n"
                  << "    fx: " << cameraMatrix[0] << "\n"
                  << "    fy: " << cameraMatrix[4] << "\n"
                  << "    cx: " << cameraMatrix[2] << "\n"
                  << "    cy: " << cameraMatrix[5];

#if defined(CUDA_ON_TARGET)
        m_cudaParameters[0] = (double)frameWidth;
        m_cudaParameters[1] = (double)frameHeight;
        m_cudaParameters[2] = (double)cameraMatrix[0]; //fx
        m_cudaParameters[3] = (double)cameraMatrix[4]; //fy
        m_cudaParameters[4] = (double)cameraMatrix[2]; //cx
        m_cudaParameters[5] = (double)cameraMatrix[5]; //cy
        cudaObj.setParameters(m_cudaParameters);
        cudaObj.buildGeometryCorrectionCache();
#else
        buildGeometryCalibrationCache(cameraMatrix, frameWidth, frameHeight);
#endif
    }

    status = getIntrinsic(DISTORTION_COEFFICIENTS, distortionCoeffs);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read distortion coefficients from eeprom";
    } else {
        LOG(INFO) << "Camera distortion coefficient parameters:\n"
                  << "    k1: " << distortionCoeffs[0] << "\n"
                  << "    k2: " << distortionCoeffs[1] << "\n"
                  << "    k3: " << distortionCoeffs[2];

#if defined(CUDA_ON_TARGET)
        m_cudaParameters[0] = (double)frameWidth;
        m_cudaParameters[1] = (double)frameHeight;
        m_cudaParameters[2] = (double)cameraMatrix[0];     //fx
        m_cudaParameters[3] = (double)cameraMatrix[4];     //fy
        m_cudaParameters[4] = (double)cameraMatrix[2];     //cx
        m_cudaParameters[5] = (double)cameraMatrix[5];     //cy
        m_cudaParameters[6] = (double)distortionCoeffs[0]; //k1
        m_cudaParameters[7] = (double)distortionCoeffs[1]; //k2
        m_cudaParameters[8] = (double)distortionCoeffs[2]; //k3
        m_cudaParameters[9] = (double)cameraMatrix[2];     //x0
        m_cudaParameters[10] = (double)cameraMatrix[5];    //y0
        cudaObj.setParameters(m_cudaParameters);
        cudaObj.buildDistortionCorrectionCache();
#else
        buildDistortionCorrectionCache(frameWidth, frameHeight);

#endif
    }

    /*Execute the mode change command*/
    uint16_t afeRegsAddr[] = {0x4000, 0x4001, 0x7c22};
    uint16_t afeRegsVal[] = {mode_id, 0x0004, 0x0004};
    depthSensor->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);

    return Status::OK;
}

//! calibrateDepth - Calibrate the depth data
/*!
calibrateDepth - Calibrate the depth data using the gain and offset
\param frame - Buffer with the depth data, used to return the calibrated data
\param frame_size - Number of samples in the frame data
*/
aditof::Status Calibration3D_Smart::calibrateDepth(uint16_t *frame,
                                                   uint32_t frame_size) {

#if defined(CUDA_ON_TARGET)
    cudaObj.cpyFrameToGPU((uint16_t *)frame);
    cudaObj.applyDepthCorrection();
    cudaObj.cpyFrameFromGPU(frame);
    return aditof::Status::OK;
#else
    using namespace aditof;

    uint16_t *cache = m_depth_cache;

    uint16_t *end = frame + (frame_size - frame_size % 8);
    uint16_t *framePtr = frame;

    for (; framePtr < end; framePtr += 8) {
        *framePtr = *(cache + *framePtr);
        *(framePtr + 1) = *(cache + *(framePtr + 1));
        *(framePtr + 2) = *(cache + *(framePtr + 2));
        *(framePtr + 3) = *(cache + *(framePtr + 3));
        *(framePtr + 4) = *(cache + *(framePtr + 4));
        *(framePtr + 5) = *(cache + *(framePtr + 5));
        *(framePtr + 6) = *(cache + *(framePtr + 6));
        *(framePtr + 7) = *(cache + *(framePtr + 7));
    }

    end += (frame_size % 8);

    for (; framePtr < end; framePtr++) {
        *framePtr = *(cache + *framePtr);
    }

    return Status::OK;

#endif
}

//! calibrateCameraGeometry - Compensate for lens distorsion in the depth data
/*!
calibrateCameraGeometry - Compensate for lens distorsion in the depth data
\param frame - Buffer with the depth data, used to return the calibrated data
\param frame_size - Number of samples in the frame data
*/
aditof::Status
Calibration3D_Smart::calibrateCameraGeometry(uint16_t *frame,
                                             uint32_t frame_size) {

#if defined(CUDA_ON_TARGET)
    cudaObj.cpyFrameToGPU((uint16_t *)frame);
    cudaObj.applyGeometryCorrection();
    cudaObj.cpyFrameFromGPU(frame);
    return aditof::Status::OK;
#else
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
#endif
}

// Create a cache to speed up depth calibration computation
void Calibration3D_Smart::buildDepthCalibrationCache(float gain, float offset,
                                                     int16_t maxPixelValue,
                                                     int range) {
    if (m_depth_cache) {
        delete[] m_depth_cache;
    }

    m_depth_cache = new uint16_t[maxPixelValue + 1];
    for (int16_t current = 0; current <= maxPixelValue; ++current) {
        int16_t currentValue =
            static_cast<int16_t>(static_cast<float>(current) * gain + offset);
        m_depth_cache[current] = currentValue <= range ? currentValue : range;
    }
    // std::cout << "CPU depth: \n";
    // for (int i = 0; i < 10; i++) {
    //     std::cout << m_depth_cache[i] << ", ";
    // }
    // std::cout << "\n\n\n";
}

// Create a cache to speed up depth geometric camera calibration computation
void Calibration3D_Smart::buildGeometryCalibrationCache(
    const std::vector<float> &cameraMatrix, unsigned int width,
    unsigned int height) {

    float fx = cameraMatrix[0];
    float fy = cameraMatrix[4];
    float x0 = cameraMatrix[2];
    float y0 = cameraMatrix[5];

    const bool validParameters = (fx != 0 && fy != 0);

    if (m_geometry_cache) {
        delete[] m_geometry_cache;
    }

    m_geometry_cache = new double[width * height];
    for (uint16_t i = 0; i < height; i++) {
        for (uint16_t j = 0; j < width; j++) {

            if (validParameters) {
                double tanXAngle = (x0 - j) / fx;
                double tanYAngle = (y0 - i) / fy;

                m_geometry_cache[i * width + j] =
                    1.0 /
                    sqrt(1 + tanXAngle * tanXAngle + tanYAngle * tanYAngle);
            } else {
                m_geometry_cache[i * width + j] = 1;
            }
        }
    }
    // std::cout << "CPU geometry: \n";
    // for (int i = 0; i < 10; i++) {
    //     std::cout << m_geometry_cache[i] << ", ";
    // }
    // std::cout << "\n\n\n";
}

void Calibration3D_Smart::buildDistortionCorrectionCache(unsigned int width,
                                                         unsigned int height) {
    using namespace aditof;

    double fx = (double)m_intrinsics[2];
    double fy = (double)m_intrinsics[3];
    double cx = (double)m_intrinsics[0];
    double cy = (double)m_intrinsics[1];
    std::vector<double> k{(double)m_intrinsics[4], (double)m_intrinsics[5], 0.0,
                          0.0, m_intrinsics[6]};
    if (m_distortion_cache) {
        delete[] m_distortion_cache;
    }

    m_distortion_cache = new double[width * height];
    for (uint16_t i = 0; i < width; i++) {
        for (uint16_t j = 0; j < height; j++) {
            double x = (i - cx) / fx;
            double y = (j - cy) / fy;

            //DISTORTION_COEFFICIENTS for [k1, k2, p1, p2, k3]
            double r2 = x * x + y * y;
            double k_calc =
                double(1 + k[0] * r2 + k[1] * r2 * r2 + k[4] * r2 * r2 * r2);
            m_distortion_cache[j * width + i] = k_calc;
        }
    }

    std::cout << "CPU distortion: \n";
    for (int i = 0; i < 10; i++) {
        std::cout << m_distortion_cache[i] << ", ";
    }
    std::cout << "\n\n\n";
}

aditof::Status Calibration3D_Smart::distortionCorrection(uint16_t *frame,
                                                         unsigned int width,
                                                         unsigned int height) {

#if defined(CUDA_ON_TARGET)
    cudaObj.cpyFrameToGPU((uint16_t *)frame);
    cudaObj.applyDistortionCorrection();
    cudaObj.cpyFrameFromGPU(frame);
    return aditof::Status::OK;
#else

    using namespace aditof;
    double fx = (double)m_intrinsics[2];
    double fy = (double)m_intrinsics[3];
    double cx = (double)m_intrinsics[0];
    double cy = (double)m_intrinsics[1];
    uint16_t *buff;

    buff = new uint16_t[width * height];

    for (uint16_t i = 0; i < width; i++) {
        for (uint16_t j = 0; j < height; j++) {
            //transform in dimensionless space
            double x = (double(i) - cx) / fx;
            double y = (double(j) - cy) / fy;

            //apply correction
            double x_dist_adim = x * m_distortion_cache[j * width + i];
            double y_dist_adim = y * m_distortion_cache[j * width + i];

            //back to original space
            int x_dist = (int)(x_dist_adim * fx + cx);
            int y_dist = (int)(y_dist_adim * fy + cy);

            if (x_dist >= 0 && x_dist < (int)width && y_dist >= 0 &&
                y_dist < (int)height) {
                buff[j * width + i] = frame[y_dist * width + x_dist];
            } else {
                buff[j * width + i] = frame[j * width + i];
            }
        }
    }
    memcpy(frame, buff, width * height * 2);
    delete[] buff;
    return Status::OK;
#endif
}
