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
#include "connections/usb/usb_temperature_sensor.h"
#include "usb_windows_utils.h"

#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

using namespace aditof;

struct UsbTemperatureSensor::ImplData {
    struct UsbHandle *handle;
    std::string name;
    unsigned char id;
};

UsbTemperatureSensor::UsbTemperatureSensor(const std::string &name,
                                           unsigned char id)
    : m_implData(new ImplData) {
    m_implData->handle = nullptr;
    m_implData->name = name;
    m_implData->id = id;
}

UsbTemperatureSensor::~UsbTemperatureSensor() = default;

Status UsbTemperatureSensor::open(void *handle) {
    if (!handle) {
        LOG(ERROR) << "Invalid handle";
        return Status::INVALID_ARGUMENT;
    }

    m_implData->handle = reinterpret_cast<struct UsbHandle *>(handle);

    return Status::OK;
}

Status UsbTemperatureSensor::read(float &temperature) {
    if (!m_implData->handle) {
        LOG(ERROR) << "Cannot read! Temperature sensor is not opened.";
        return Status::GENERIC_ERROR;
    }

    ExUnitHandle handle;

    HRESULT hr = UsbWindowsUtils::UvcFindNodeAndGetControl(
        &handle, &m_implData->handle->pVideoInputFilter);
    if (hr != S_OK) {
        LOG(WARNING) << "Failed to find node and get control. Error: "
                     << std::hex << hr;
        return Status::GENERIC_ERROR;
    }

    hr = UsbWindowsUtils::UvcExUnitSetProperty(
        &handle, 3, reinterpret_cast<const uint8_t *>(&m_implData->id), 4);
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed to set property via UVC extension unit. Error: "
                     << std::hex << hr;
        return Status::GENERIC_ERROR;
    }

    hr = UsbWindowsUtils::UvcExUnitGetProperty(
        &handle, 3, reinterpret_cast<uint8_t *>(&temperature), 4);
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed to get property via UVC extension unit. Error: "
                     << std::hex << hr;
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status UsbTemperatureSensor::close() {
    m_implData->handle = nullptr;

    return Status::OK;
}

Status UsbTemperatureSensor::getName(std::string &name) const {
    name = m_implData->name;
    return Status::OK;
}
