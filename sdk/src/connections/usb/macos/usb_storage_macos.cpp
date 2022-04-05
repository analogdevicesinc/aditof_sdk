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
/* This is an empty implementation and it's purpose is to give the UsbStorage
 * a default implementation on platforms where it is not used (Windows, Linux,
 * MacOS).
 */

#include "connections/usb/usb_storage.h"

using namespace aditof;

struct UsbStorage::ImplData {};

UsbStorage::UsbStorage(const std::string &name, unsigned char id) {}

UsbStorage::~UsbStorage() = default;

aditof::Status UsbStorage::open(void *handle) {
    // TO DO when enabling macos support
    return Status::UNAVAILABLE;
}

aditof::Status UsbStorage::read(const uint32_t address, uint8_t *data,
                                const size_t bytesCount) {
    // TO DO when enabling macos support
    return Status::UNAVAILABLE;
}

aditof::Status UsbStorage::write(const uint32_t address, const uint8_t *data,
                                 const size_t bytesCount) {
    // TO DO when enabling macos support
    return Status::UNAVAILABLE;
}

aditof::Status UsbStorage::close() {
    // TO DO when enabling macos support
    return Status::UNAVAILABLE;
}

aditof::Status UsbStorage::getName(std::string &name) const {
    // TO DO when enabling macos support
    return Status::UNAVAILABLE;
}
