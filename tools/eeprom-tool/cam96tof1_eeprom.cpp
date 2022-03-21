/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Analog Devices, Inc.
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
#include "cam96tof1_eeprom.h"
#include "eeprom_map.h"

#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

Camera96Tof1Eeprom::Camera96Tof1Eeprom(
    std::shared_ptr<aditof::StorageInterface> _eeprom)
    : m_eeprom(_eeprom) {
    if (!_eeprom) {
        LOG(ERROR) << "null pointer to EEPROM.";
    }
}

aditof::Status Camera96Tof1Eeprom::read(std::vector<uint8_t> &data) {
    float read_size = 100;
    uint32_t eepromSize;
    aditof::Status status;
    std::string eepromName;

    m_eeprom->getName(eepromName);
    eepromSize = EEPROMS.at(eepromName).size;

    //read size
    m_eeprom->read((uint32_t)0, (uint8_t *)&read_size, (size_t)4);
    LOG(INFO) << "EEPROM calibration data size " << read_size << " bytes";
    if (read_size > eepromSize) {
        LOG(WARNING) << "Invalid calibration data size";
        return aditof::Status::GENERIC_ERROR;
    }

    //read data
    data.resize(read_size);
    status = m_eeprom->read(4, data.data(), read_size);
    if (status != aditof::Status::OK) {
        data.resize(0);
        LOG(WARNING) << "Failed to read from EEPROM";
        return status;
    }

    LOG(ERROR) << "Successfully read data from EEPROM";

    return aditof::Status::OK;
}

aditof::Status Camera96Tof1Eeprom::write(const std::vector<uint8_t> &data) {
    aditof::Status status;
    float size = static_cast<float>(data.size());

    status = m_eeprom->write((uint32_t)0, (uint8_t *)&size, (size_t)4);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "failed to write to EEPROM";
        return status;
    }

    status = m_eeprom->write((uint32_t)4, (uint8_t *)data.data(), (size_t)size);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "failed to write to EEPROM";
        return status;
    }

    LOG(INFO) << "Successfully wrote data to EEPROM";

    return aditof::Status::OK;
}
