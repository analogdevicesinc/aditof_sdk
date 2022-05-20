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
#include "calibration_96tof1.h"
#include "aditof/storage_interface.h"

#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <math.h>

#define EEPROM_SIZE 131072

Calibration96Tof1::Calibration96Tof1()
    : m_intrinsics(nullptr), m_distCoeffs(nullptr), m_depth_cache(nullptr),
      m_distortion_cache(nullptr), m_geometry_cache(nullptr), m_range(16000) {
    std::unordered_map<float, param_struct> Header;
    Header[EEPROM_VERSION].value = {0};
    Header[EEPROM_VERSION].size =
        sizeof(Header[EEPROM_VERSION].value.size()) * 4;
    Header[TOTAL_SIZE].value = {1000};
    Header[TOTAL_SIZE].size = sizeof(Header[TOTAL_SIZE].value.size()) * 4;
    Header[NUMBER_OF_MODES].value = {3};
    Header[NUMBER_OF_MODES].size = sizeof(Header[TOTAL_SIZE].value.size()) * 4;

    std::unordered_map<float, param_struct> CameraIntrinsic;
    CameraIntrinsic[EEPROM_VERSION].value = {0};
    CameraIntrinsic[EEPROM_VERSION].size =
        (uint32_t)(CameraIntrinsic[EEPROM_VERSION].value.size() * 4);
    CameraIntrinsic[CAL_SER_NUM].value = {0};
    CameraIntrinsic[CAL_SER_NUM].size =
        (uint32_t)(CameraIntrinsic[CAL_SER_NUM].value.size() * 4);
    CameraIntrinsic[CAL_DATE].value = {12042019};
    CameraIntrinsic[CAL_DATE].size =
        (uint32_t)(CameraIntrinsic[CAL_SER_NUM].value.size() * 4);
    CameraIntrinsic[INTRINSIC].value = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    CameraIntrinsic[INTRINSIC].size =
        (uint32_t)(CameraIntrinsic[INTRINSIC].value.size() * 4);

    m_calibration_map[HEADER].size = (uint32_t)getPacketSize(Header);
    m_calibration_map[HEADER].packet = Header;
    m_calibration_map[CAMERA_INTRINSIC].size =
        (uint32_t)getPacketSize(CameraIntrinsic);
    m_calibration_map[CAMERA_INTRINSIC].packet = CameraIntrinsic;
    m_calibration_map[HEADER].packet[TOTAL_SIZE].value = {
        getMapSize(m_calibration_map)};
}

Calibration96Tof1::~Calibration96Tof1() {
    if (m_depth_cache) {
        delete[] m_depth_cache;
    }

    if (m_geometry_cache) {
        delete[] m_geometry_cache;
    }

    if (m_distortion_cache) {
        delete[] m_distortion_cache;
    }

    if (m_intrinsics) {
        delete[] m_intrinsics;
    }

    if (m_distCoeffs) {
        delete[] m_distCoeffs;
    }
}

//! DisplayCalMap - Display the entire calibration map
/*!
    DisplayCalMap - Displays the entire calibration map.
    Calibratin map is nested hash map consiting of primary key( packet type key)
    and secondary key( parameter type key)
*/
aditof::Status Calibration96Tof1::displayCalMap() const {
    using namespace aditof;

    for (const auto &mapElement : m_calibration_map) {
        std::cout << "Key: " << mapElement.first;
        const packet_struct &sub_packet_map = mapElement.second;
        std::cout << "\t Size: " << sub_packet_map.size << std::endl;

        for (const auto &packet : sub_packet_map.packet) {
            std::cout << "\tSub Key: " << packet.first;
            std::cout << "\tSub Size: " << packet.second.size;
            std::cout << "\tSub Value: ";

            for (const auto &value : packet.second.value)
                std::cout << value << " ";
            std::cout << std::endl;
        }
    }

    return Status::OK;
}

//! SaveCalMap - Save the entire calibration map
/*!
SaveCalMap - Saves the entire calibration map as binary to a file.
\eeprom - Pointer to an eeprom instance
*/
aditof::Status Calibration96Tof1::saveCalMap(
    std::shared_ptr<aditof::StorageInterface> eeprom) {
    using namespace aditof;

    std::vector<float> data;
    for (const auto &mapElement : m_calibration_map) {
        data.push_back(mapElement.first);
        const packet_struct &sub_packet_map = mapElement.second;
        data.push_back((float)sub_packet_map.size);

        for (const auto &packet : sub_packet_map.packet) {
            data.push_back(packet.first); // write parameter key
            data.push_back(
                (float)packet.second.size); // write size of parameter

            for (const auto &value : packet.second.value) {
                data.push_back(
                    static_cast<float>(value)); // write parameter values
            }
        }
    }

    float size = static_cast<float>(data.size() * sizeof(uint32_t));
    eeprom->write((uint32_t)0, (uint8_t *)&size, (size_t)4);
    eeprom->write((uint32_t)4, (uint8_t *)data.data(), (size_t)size);

    return Status::OK;
}

//! ReadCalMap - Read the entire calibration map
/*!
ReadCalMap - Read the entire calibration map from a binary file
\eeprom - Pointer to an eeprom instance
*/
aditof::Status Calibration96Tof1::readCalMap(
    std::shared_ptr<aditof::StorageInterface> eeprom) {
    using namespace aditof;

    Status status = Status::OK;
    uint8_t *data;
    float read_size = 100;
    uint32_t j = 0;
    float key;

    eeprom->read(0, (uint8_t *)&read_size, 4);
    LOG(INFO) << "EEPROM calibration data size " << read_size << " bytes";

    if (read_size > EEPROM_SIZE) {
        LOG(WARNING) << "Invalid calibration data size";
        return Status::GENERIC_ERROR;
    }

    data = (uint8_t *)malloc((size_t)read_size);

    status = eeprom->read(4, data, (size_t)read_size);
    if (status != Status::OK) {
        free(data);
        LOG(WARNING) << "Failed to read from eeprom";
        return status;
    }

    while (j < read_size) {
        key = *(float *)(data + j);
        j += 4;

        packet_struct sub_packet_map;

        sub_packet_map.size = (uint32_t) * (float *)(data + j);
        j += 4;

        for (unsigned int i = 0;
             i < sub_packet_map.size /
                     (sizeof(float));) // Parse all the sub-packets
        {
            float parameter_key;
            parameter_key =
                *(float *)(data + j); // Parse key of parameter from sub packet
            j += 4;
            i++;
            sub_packet_map.packet[parameter_key].size =
                (uint32_t) *
                (float *)(data + j); // Parse size of parameter from sub packet
            j += 4;
            i++;

            uint32_t number_elements =
                sub_packet_map.packet[parameter_key].size / sizeof(float);
            std::list<float> elements;
            for (unsigned int k = 0; k < number_elements; k++) {
                sub_packet_map.packet[parameter_key].value.push_back(
                    *(float *)(data +
                               j)); // Parse size of parameter from sub packet
                j += 4;
                i++;
            }
        }
        m_calibration_map[key].size = sub_packet_map.size;
        m_calibration_map[key].packet = sub_packet_map.packet;
    }

    free(data);

    return Status::OK;
}

//! getAfeFirmware - Get the firmware for a mode
/*!
getAfeFirmware - Get the firmware for a mode
\param mode - Camera mode
\param data - Buffer where to store the firmware
*/
aditof::Status
Calibration96Tof1::getAfeFirmware(const std::string &mode,
                                  std::vector<uint16_t> &data) const {
    using namespace aditof;

    uint8_t cal_mode;

    if (mode == "near") {
        cal_mode = 3;
    } else if (mode == "medium") {
        cal_mode = 5;
    } else if (mode == "far") {
        cal_mode = 7;
    } else {
        LOG(WARNING) << "Invalid firmware mode " << mode.c_str();
        return Status::INVALID_ARGUMENT;
    }

    for (const auto &mapElement : m_calibration_map) {
        float key = mapElement.first;
        const packet_struct &sub_packet_map = mapElement.second;

        if (cal_mode == key) {
            for (const auto &packet : sub_packet_map.packet) {
                if (packet.first == 5) {
                    for (const auto &value : packet.second.value) {
                        data.push_back(static_cast<uint16_t>(value));
                    }
                    return Status::OK;
                }
            }
        }
    }

    return Status::GENERIC_ERROR;
}

//! getGainOffset - Get the depth gain ad offset values for a mode
/*!
getGainOffset - Get the depth gain ad offset values for a mode
\param mode - Camera mode
\param gain - Stores the retuned gain value
\param offset - Stores the retuned offset value
*/
aditof::Status Calibration96Tof1::getGainOffset(const std::string &mode,
                                                float &gain,
                                                float &offset) const {
    using namespace aditof;

    uint8_t cal_mode;

    if (mode == "near") {
        cal_mode = 2;
    } else if (mode == "medium") {
        cal_mode = 4;
    } else if (mode == "far") {
        cal_mode = 6;
    } else {
        LOG(WARNING) << "Invalid firmware mode " << mode.c_str();
        return Status::INVALID_ARGUMENT;
    }

    for (const auto &mapElement : m_calibration_map) {
        float key = mapElement.first;
        const packet_struct &sub_packet_map = mapElement.second;

        if (cal_mode == key) {
            for (const auto &packet : sub_packet_map.packet) {
                if (packet.first == 26) {
                    gain = packet.second.value.front();
                }
                if (packet.first == 27) {
                    offset = packet.second.value.front();
                }
            }
            return Status::OK;
        }
    }

    return Status::GENERIC_ERROR;
}

//! getIntrinsic - Get the geometric camera calibration
/*!
getIntrinsic - Get the geometric camera calibration
\param key - Specifies which calibration values to get:
             INTRINSIC for [fx, 0, cx, 0, fy, cy, 0, 0, 1]
             DISTORTION_COEFFICIENTS for [k1, k2, p1, p2, k3]
\param data - Buffer to store the returned data
*/
aditof::Status Calibration96Tof1::getIntrinsic(float key,
                                               std::vector<float> &data) const {
    using namespace aditof;

    bool validParam = (INTRINSIC == key) || (DISTORTION_COEFFICIENTS == key);

    if (!validParam) {
        LOG(WARNING) << "Invalid intrinsic " << std::to_string(key).c_str();
        return Status::INVALID_ARGUMENT;
    }

    for (const auto &mapElement : m_calibration_map) {
        float mapKey = mapElement.first;
        const packet_struct &sub_packet_map = mapElement.second;

        if (mapKey == CAMERA_INTRINSIC) {
            for (const auto &packet : sub_packet_map.packet) {
                float packetKey = packet.first;

                if (packetKey == key) {
                    const param_struct &param_map = packet.second;
                    const std::list<float> &valList = param_map.value;

                    data.insert(data.begin(), valList.begin(), valList.end());
                    return Status::OK;
                }
            }
            LOG(WARNING) << "No intrinsics found in the device memory for key "
                         << std::to_string(key).c_str();
        }
    }

    return Status::GENERIC_ERROR;
}

//! setMode - Sets the mode to be used for depth calibration
/*!
setMode - Sets the mode to be used for depth calibration
\param mode - Camera depth mode
\param range - Max range for selected mode
\param frameWidth - Width of the depth image in pixels
\param frameHeight - Height of the depth image in pixels
*/
aditof::Status Calibration96Tof1::setMode(const std::string &mode, int range,
                                          unsigned int frameWidth,
                                          unsigned int frameHeight) {
    using namespace aditof;

    Status status = Status::OK;
    std::vector<float> cameraMatrix;
    std::vector<float> distortionCoeffs;
    const int16_t pixelMaxValue = (1 << 12) - 1; // 4095
    float gain = 1.0, offset = 0.0;

    status = getGainOffset(mode, gain, offset);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read gain and offset from eeprom";
        return status;
    } else {
        LOG(INFO) << "Camera calibration parameters for mode: " << mode
                  << " are gain: " << gain << " "
                  << "offset: " << offset;
    }
    buildDepthCalibrationCache(gain, offset, pixelMaxValue, range);
    m_range = range;

    status = getIntrinsic(INTRINSIC, cameraMatrix);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read intrinsic from eeprom";
        return status;
    } else {
        LOG(INFO) << "Camera intrinsic parameters:\n"
                  << "    fx: " << cameraMatrix[0] << "\n"
                  << "    fy: " << cameraMatrix[4] << "\n"
                  << "    cx: " << cameraMatrix[2] << "\n"
                  << "    cy: " << cameraMatrix[5];
    }
    buildGeometryCalibrationCache(cameraMatrix, frameWidth, frameHeight);

    status = getIntrinsic(DISTORTION_COEFFICIENTS, distortionCoeffs);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read distortion coeffs from eeprom";
        return status;
    }
    buildDistortionCorrectionCache(cameraMatrix, distortionCoeffs, frameWidth,
                                   frameHeight);

    return status;
}

//! calibrateDepth - Calibrate the depth data
/*!
calibrateDepth - Calibrate the depth data using the gain and offset
\param frame - Buffer with the depth data, used to return the calibrated data
\param frame_size - Number of samples in the frame data
*/
aditof::Status Calibration96Tof1::calibrateDepth(uint16_t *frame,
                                                 uint32_t frame_size) {
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
}

//! calibrateCameraGeometry - Compensate for lens distorsion in the depth data
/*!
calibrateCameraGeometry - Compensate for lens distorsion in the depth data
\param frame - Buffer with the depth data, used to return the calibrated data
\param frame_size - Number of samples in the frame data
*/
aditof::Status Calibration96Tof1::calibrateCameraGeometry(uint16_t *frame,
                                                          uint32_t frame_size) {
    using namespace aditof;

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

// Create a cache to speed up depth calibration computation
void Calibration96Tof1::buildDepthCalibrationCache(float gain, float offset,
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
}

// Create a cache to speed up depth geometric camera calibration computation
void Calibration96Tof1::buildGeometryCalibrationCache(
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

    if (!validParameters) {
        LOG(WARNING) << "Invalid intrinsic parameters fx or fy are 0. No "
                        "correction will be applied!";
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
}

// Calculate and return the total size of calibration map
float Calibration96Tof1::getMapSize(
    const std::unordered_map<float, packet_struct> &calibration_map) const {
    float total_size = 0;
    // Calculate total size of calibration map
    for (const auto &mapElement : calibration_map) {
        total_size += mapElement.second.size; // Add size of all the sub packets
    }
    return total_size;
}

// Calculate and return the size of a packet
float Calibration96Tof1::getPacketSize(
    const std::unordered_map<float, param_struct> &packet) const {
    float packet_size = 0;
    for (const auto &mapElement : packet) {
        packet_size +=
            mapElement.second.size + 8; // Added 8 for size of key and size
    }
    return packet_size;
}

void Calibration96Tof1::buildDistortionCorrectionCache(
    const std::vector<float> &cameraMatrix,
    const std::vector<float> &distortionCoeffs, unsigned int width,
    unsigned int height) {
    using namespace aditof;

    //DISTORTION_COEFFICIENTS for [k1, k2, p1, p2, k3]
    m_distCoeffs = new double[5];
    m_intrinsics = new double[4];
    for (int i = 0; i < 5; i++) {
        m_distCoeffs[i] = double(distortionCoeffs.at(i));
    }

    m_intrinsics[0] = double(cameraMatrix[0]);
    m_intrinsics[1] = double(cameraMatrix[4]);
    m_intrinsics[2] = double(cameraMatrix[2]);
    m_intrinsics[3] = double(cameraMatrix[5]);

    double fx = m_intrinsics[0];
    double fy = m_intrinsics[1];
    double cx = m_intrinsics[2];
    double cy = m_intrinsics[3];

    if (m_distortion_cache) {
        delete[] m_distortion_cache;
    }

    m_distortion_cache = new double[width * height];
    for (uint16_t i = 0; i < width; i++) {
        for (uint16_t j = 0; j < height; j++) {
            double x = (i - cx) / fx;
            double y = (j - cy) / fy;

            double r2 = x * x + y * y;
            double k_calc =
                double(1 + m_distCoeffs[0] * r2 + m_distCoeffs[1] * r2 * r2 +
                       m_distCoeffs[4] * r2 * r2 * r2);
            m_distortion_cache[j * width + i] = k_calc;
        }
    }
}

aditof::Status Calibration96Tof1::distortionCorrection(uint16_t *frame,
                                                       unsigned int width,
                                                       unsigned int height) {
    using namespace aditof;

    double fx = m_intrinsics[0];
    double fy = m_intrinsics[1];
    double cx = m_intrinsics[2];
    double cy = m_intrinsics[3];

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

    for (uint16_t i = 0; i < width; i++) {
        for (uint16_t j = 0; j < height; j++) {
            //transform in dimensionless space
            double x = (double(i) - cx) / fx;
            double y = (double(j) - cy) / fy;
            double r2 = x * x + y * y;

            //apply correction
            double x_dist_adim = x + (2 * m_distCoeffs[2] * x * y +
                                      m_distCoeffs[3] * (r2 + 2 * x * x));
            double y_dist_adim = y + (m_distCoeffs[2] * (r2 + 2 * y * y) +
                                      2 * m_distCoeffs[3] * x * y);

            //back to original space
            int x_dist = (int)(x_dist_adim * fx + cx);
            int y_dist = (int)(y_dist_adim * fy + cy);

            if (x_dist >= 0 && x_dist < (int)width && y_dist >= 0 &&
                y_dist < (int)height) {
                frame[j * width + i] = buff[y_dist * width + x_dist];
            } else {
                frame[j * width + i] = buff[j * width + i];
            }
        }
    }

    delete[] buff;
    return Status::OK;
}
