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
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <aditof/device_interface.h>
#include <aditof/status_definitions.h>
#include <iostream>
#include <list>
#include <memory>
#include <stdint.h>
#include <unordered_map>

// Hashmap key for Packet type
#define HEADER 0
#define CAMERA_INTRINSIC 1

// Hashmap key for common parameters
#define EEPROM_VERSION 1
#define CAL_SER_NUM 2
#define CAL_DATE 3
#define CHECKSUM 4

// Hashmap key for Header Parameters
#define TOTAL_SIZE 5
#define NUMBER_OF_MODES 6

// Hashmap key for Camera Intrinsic
#define INTRINSIC 5
#define DISTORTION_COEFFICIENTS 6

//! param_struct - Structure to hold the value of parameters
/*!
    param_struct provides structure to store the value of parameters.
    \param size - size of the parameter value
    \param value - the value of the parameter
*/
struct param_struct {
    uint32_t size;
    // float value; Fixed 1 value per key
    std::list<float> value; // list of value per key
};

//! packet_struct - Structure to hold the packet consisting of map of parameters
/*!
    packet_struct provides structure to hold the packet(sub map) of parameters
    \param size - size of the packet
    \param packet - the packet(sub map) for parameters of certain packet type
*/
struct packet_struct {
    uint32_t size;
    std::unordered_map<float, param_struct> packet;
};

class Calibration {
  public:
    Calibration();
    ~Calibration();

  public:
    aditof::Status saveCalMap(std::shared_ptr<aditof::DeviceInterface> device);
    aditof::Status readCalMap(std::shared_ptr<aditof::DeviceInterface> device);
    aditof::Status displayCalMap() const;
    aditof::Status getAfeFirmware(const std::string &mode,
                                  std::vector<uint16_t> &data) const;
    aditof::Status getGainOffset(const std::string &mode, float &gain,
                                 float &offset) const;
    aditof::Status getIntrinsic(float key, std::vector<float> &data) const;

  private:
    float getMapSize(
        const std::unordered_map<float, packet_struct> &calibration_map) const;
    float
    getPacketSize(const std::unordered_map<float, param_struct> &packet) const;

  private:
    std::unordered_map<float, packet_struct> m_calibration_map;
};

#endif /*CALIBRATION_H*/
