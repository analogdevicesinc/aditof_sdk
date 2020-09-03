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
#include "usb_eeprom.h"
#include "usb_windows_utils.h"

#include <glog/logging.h>
#include <string>

using namespace aditof;

struct UsbEeprom::ImplData {
    struct UsbHandle *handle;
    std::string name;
    std::string driverPath;
};

UsbEeprom::UsbEeprom() : m_implData(new ImplData) {
    m_implData->handle = nullptr;
}

Status UsbEeprom::open(void *handle, const std::string &name,
                       const std::string &driver_path) {
    if (!handle) {
        LOG(ERROR) << "Invalid handle";
        return Status::INVALID_ARGUMENT;
    }
    m_implData->handle = reinterpret_cast<struct UsbHandle *>(handle);
    m_implData->name = name;
    m_implData->driverPath = driver_path;

    return Status::OK;
}

Status UsbEeprom::read(const uint32_t address, uint8_t *data,
                       const size_t bytesCount) {
    if (!m_implData->handle) {
        LOG(ERROR) << "Cannot read! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot read! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

    HRESULT hr = UsbWindowsUtils::UvcExUnitReadBuffer(
        m_implData->handle->pVideoInputFilter, 5, address, data, bytesCount);
    if (FAILED(hr)) {
        LOG(WARNING)
            << "Failed to read buffer through UVC extension unit. Error: "
            << hr;
        return Status::GENERIC_ERROR;
    }
    return Status::OK;
}

Status UsbEeprom::write(const uint32_t address, const uint8_t *data,
                        const size_t bytesCount) {
    if (!m_implData->handle) {
        LOG(ERROR) << "Cannot write! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot write! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

    HRESULT hr = UsbWindowsUtils::UvcExUnitWriteBuffer(
        m_implData->handle->pVideoInputFilter, 6, address, data, bytesCount);
    if (FAILED(hr)) {
        LOG(WARNING)
            << "Failed to write buffer through UVC extension unit. Error: "
            << hr;
        return Status::GENERIC_ERROR;
    }
    return Status::OK;
}

Status UsbEeprom::close() {
    m_implData->handle = nullptr;
    m_implData->name.clear();
    m_implData->driverPath.clear();

    return Status::OK;
}

Status UsbEeprom::getName(std::string &name) {
    name = m_implData->name;
    return Status::OK;
}
