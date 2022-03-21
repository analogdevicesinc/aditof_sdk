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
#include "eeprom.h"
#include "i2c_common.h"

#include <errno.h>
#include <fcntl.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <linux/fs.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

using namespace aditof;

struct Eeprom::ImplData {
    eeprom eepromDev;
    std::string name;
    std::string driverPath;
};

Eeprom::Eeprom(const std::string &driverName, const std::string &driverPath)
    : m_implData(new ImplData) {
    m_implData->name = driverName;
    m_implData->driverPath = driverPath;
}

Eeprom::~Eeprom() = default;

Status Eeprom::open(void *) {
    eeprom *e = &m_implData->eepromDev;

    e->fd = fopen(m_implData->driverPath.c_str(), "w+");
    if (!e->fd) {
        LOG(ERROR) << "fopen() failed. Error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    fseek(e->fd, 0x0, SEEK_END);
    long len = ftell(e->fd);
    if (len < 0) {
        LOG(ERROR) << "ftell() failed. Error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }
    e->length = static_cast<unsigned int>(len);
    fseek(e->fd, 0x0, SEEK_SET);

    return Status::OK;
}

Status Eeprom::read(const uint32_t address, uint8_t *data,
                    const size_t bytesCount) {
    auto fd = m_implData->eepromDev.fd;

    if (!fd) {
        LOG(ERROR) << "Cannot read! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot read! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

    fseek(fd, address, SEEK_SET);
    size_t ret = fread(data, 1, bytesCount, fd);
    if (ret < bytesCount) {
        LOG(ERROR) << "fread() failed. Error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }
    return Status::OK;
}

Status Eeprom::write(const uint32_t address, const uint8_t *data,
                     const size_t bytesCount) {
    auto fd = m_implData->eepromDev.fd;

    if (!fd) {
        LOG(ERROR) << "Cannot write! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot write! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

    fseek(fd, address, SEEK_SET);
    size_t ret = fwrite(data, 1, bytesCount, fd);
    if (ret < bytesCount) {
        LOG(ERROR) << "fwrite() failed. Error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }
    return Status::OK;
}

Status Eeprom::close() {
    if (m_implData->eepromDev.fd) {
        fclose(m_implData->eepromDev.fd);
        m_implData->eepromDev.fd = nullptr;
    }

    return Status::OK;
}

Status Eeprom::getName(std::string &name) const {
    name = m_implData->name;
    return Status::OK;
}
