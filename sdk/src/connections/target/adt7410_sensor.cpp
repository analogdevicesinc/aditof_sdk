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
#include "adt7410_sensor.h"

#include <errno.h>
#include <fcntl.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#include <cstring>
#include <unistd.h>
#endif
#include <linux/fs.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define TEMPERATURE_MSB_REG 0x0
#define TEMPERATURE_LSB_REG 0x1
#define CONFIGURATION_REG 0x3

using namespace aditof;

struct ADT7410::ImplData {
    temp_sensor tDev;
    std::string driverPath;
    int i2c_address;
};

ADT7410::ADT7410(const std::string &name, const std::string &driver_path,
                 int i2c_address)
    : m_implData(new ImplData), m_name(name) {
    m_implData->driverPath = driver_path;
    m_implData->i2c_address = i2c_address;
}

ADT7410::~ADT7410() = default;

Status ADT7410::open(void *) {
    if (sensor_open(m_implData->driverPath.c_str(), m_implData->i2c_address,
                    &m_implData->tDev) < 0) {
        LOG(ERROR) << "Temperature sensor open error";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status ADT7410::read(float &temperature) {
    auto fd = m_implData->tDev.fd;

    if (!fd) {
        LOG(ERROR) << "Cannot read! Temperature sensor is not opened.";
        return Status::GENERIC_ERROR;
    }

    if (sensor_read(&m_implData->tDev, &temperature) == -1) {
        LOG(ERROR) << "temperature sensor read error";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status ADT7410::close() {
    if (m_implData->tDev.fd >= 0) {
        sensor_close(&m_implData->tDev);
    }

    return Status::OK;
}

Status ADT7410::getName(std::string &name) const {
    name = m_name;

    return Status::OK;
}

int ADT7410::sensor_read(temp_sensor *t, float *temp_val) {

    if (t == nullptr) {
        LOG(ERROR) << "Received temp_sensor t null pointer";
        return -1;
    }

    if (temp_val == nullptr) {
        LOG(ERROR) << "Received temp_val null pointer";
        return -1;
    }

    short temp_reg_val, conf_reg_val;
    int ret;

    /* Read the configuration register*/
    conf_reg_val = read_byte_data(t, CONFIGURATION_REG);
    if (conf_reg_val < 0) {
        return -1;
    }

    /* Read the temperature value MSB register*/
    ret = read_byte_data(t, TEMPERATURE_MSB_REG);
    if (ret < 0) {
        return -1;
    }
    temp_reg_val = ret << 8;

    /* Read the temperature LSB register */
    ret = read_byte_data(t, TEMPERATURE_LSB_REG);
    if (ret < 0) {
        return -1;
    }
    temp_reg_val |= ret;

    /* Compute the temperature in degC */
    if (conf_reg_val & 0x80) {
        if (temp_reg_val & 0x1000) {
            *temp_val = (temp_reg_val - 65536) / 128.0;
        } else {
            *temp_val = temp_reg_val / 128.0;
        }
    } else {
        temp_reg_val = temp_reg_val >> 0x3;
        if (temp_reg_val & 0x1000) {
            *temp_val = (temp_reg_val - 8192) / 16.0;
        } else {
            *temp_val = temp_reg_val / 16.0;
        }
    }

    return 0;
}

int ADT7410::sensor_open(const char *dev_fqn, int addr, temp_sensor *t) {

    if (dev_fqn == nullptr) {
        LOG(ERROR) << "Received dev_fqn null pointer";
        return -1;
    }

    if (t == nullptr) {
        LOG(ERROR) << "Received temp_sensor t null pointer";
        return -1;
    }

    int fd, r;
    t->fd = t->addr = 0;
    t->dev = NULL;

    fd = ::open(dev_fqn, O_RDWR);
    if (fd <= 0) {
        fprintf(stderr, "Error in temperature sensor open: %s\n",
                strerror(errno));
        return -1;
    }

    // set working device
    if ((r = ioctl(fd, I2C_SLAVE, addr)) < 0) {
        fprintf(stderr, "Error temperature sensor open: %s\n", strerror(errno));
        return -1;
    }
    t->fd = fd;
    t->addr = addr;
    t->dev = dev_fqn;

    return 0;
}

int ADT7410::sensor_close(temp_sensor *t) {
    ::close(t->fd);
    t->fd = -1;
    t->dev = NULL;

    return 0;
}

int ADT7410::read_byte_data(temp_sensor *t, __u16 addr_reg) {
    int val;
    val = i2c_smbus_read_byte_data(t->fd, addr_reg);

    return val;
}
