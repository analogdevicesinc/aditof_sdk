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
#include "i2c_temperature_sensor.h"
#include "temp_sensor.h"

#include <errno.h>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/fs.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

using namespace aditof;

struct I2CTemperatureSensor::ImplData {
    temp_sensor tDev;
    std::string name;
    std::string driverPath;
    int i2c_address;
};

I2CTemperatureSensor::I2CTemperatureSensor(const std::string &name,
                                           const std::string &driver_path,
                                           int i2c_address)
    : m_implData(new ImplData) {
    m_implData->name = name;
    m_implData->driverPath = driver_path;
    m_implData->i2c_address = i2c_address;
}

I2CTemperatureSensor::~I2CTemperatureSensor() = default;

Status I2CTemperatureSensor::open(void *) {
    if (temp_sensor_open(m_implData->driverPath.c_str(),
                         m_implData->i2c_address, &m_implData->tDev) < 0) {
        LOG(ERROR) << "Temperature sensor open error";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status I2CTemperatureSensor::read(float &temperature) {
    auto fd = m_implData->tDev.fd;

    if (!fd) {
        LOG(ERROR) << "Cannot read! Temperature sensor is not opened.";
        return Status::GENERIC_ERROR;
    }

    if (temp_sensor_read(&m_implData->tDev, &temperature) == -1) {
        LOG(ERROR) << "temperature sensor read error";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status I2CTemperatureSensor::close() {
    if (m_implData->tDev.fd >= 0) {
        temp_sensor_close(&m_implData->tDev);
    }

    return Status::OK;
}

Status I2CTemperatureSensor::getName(std::string &name) const {
    name = m_implData->name;
    return Status::OK;
}
