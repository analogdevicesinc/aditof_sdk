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
#include "connections/usb/usb_storage.h"
#include "usb_linux_utils.h"
#include "utils.h"

#include <chrono>
#include <glog/logging.h>
#include <thread>

using namespace aditof;

struct UsbStorage::ImplData {
    int fd;
    std::string name;
    unsigned char id;
};

UsbStorage::UsbStorage(const std::string &name, unsigned char id)
    : m_implData(new ImplData) {
    m_implData->fd = -1;
    m_implData->name = name;
    m_implData->id = id;
}

UsbStorage::~UsbStorage() = default;

Status UsbStorage::open(void *handle) {
    if (!handle) {
        LOG(ERROR) << "Invalid handle";
        return Status::INVALID_ARGUMENT;
    }

    m_implData->fd = *(reinterpret_cast<int *>(handle));

    return Status::OK;
}

Status UsbStorage::read(const uint32_t address, uint8_t *data,
                        const size_t bytesCount) {
    if (!m_implData->fd) {
        LOG(ERROR) << "Cannot read! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot read! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

    int ret = UsbLinuxUtils::uvcExUnitReadBuffer(
        m_implData->fd, 5, m_implData->id, address, data, bytesCount);
    if (ret < 0) {
        LOG(WARNING)
            << "Failed to read buffer through UVC extension unit. Error: "
            << ret;
        return Status::GENERIC_ERROR;
    }
    return Status::OK;
}

Status UsbStorage::write(const uint32_t address, const uint8_t *data,
                         const size_t bytesCount) {

    if (!m_implData->fd) {
        LOG(ERROR) << "Cannot write! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot write! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

    int ret = UsbLinuxUtils::uvcExUnitWriteBuffer(
        m_implData->fd, 6, m_implData->id, address, data, bytesCount);
    if (ret < 0) {
        LOG(WARNING)
            << "Failed to write buffer through UVC extension unit. Error: "
            << ret;
        return Status::GENERIC_ERROR;
    }

    uint8_t eepromWriteStatus = 0;
    int attempts = 300;

    while (eepromWriteStatus == 0 && attempts > 0) {
        ret = UsbLinuxUtils::uvcExUnitReadOnePacket(
            m_implData->fd, 7, nullptr, 0, &eepromWriteStatus, 1, 1);
        --attempts;
        if (ret < 0) {
            LOG(WARNING)
                << "Failed to read a packet via UVC extension unit. Error: "
                << ret;
            return Status::GENERIC_ERROR;
        }

        if (!eepromWriteStatus) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (attempts == 0 && eepromWriteStatus == 0) {
        LOG(WARNING) << "Write operation failed. Target is in a state where "
                        "EEPROM write operations cannot be done.";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status UsbStorage::close() {
    m_implData->fd = -1;

    return Status::OK;
}

Status UsbStorage::getName(std::string &name) const {
    name = m_implData->name;
    return Status::OK;
}
