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
#include "utils_linux.h"

#include <glog/logging.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

using namespace aditof;

struct UsbEeprom::ImplData {
    int fd;
    std::string name;
    std::string driverPath;
};

UsbEeprom::UsbEeprom() : m_implData(new ImplData) { m_implData->fd = -1; }

Status UsbEeprom::open(void *handle, const char *name,
                       const char *driver_path) {
    if (!handle) {
        LOG(ERROR) << "Invalid handle";
        return Status::INVALID_ARGUMENT;
    }

    m_implData->fd = *(reinterpret_cast<int *>(handle));
    m_implData->name = name;
    m_implData->driverPath = driver_path;

    return Status::OK;
}

Status UsbEeprom::read(const uint32_t address, uint8_t *data,
                       const size_t bytesCount) {
    struct uvc_xu_control_query cq;
    uint8_t packet[MAX_BUF_SIZE];
    size_t readBytes = 0;
    size_t readLength = 0;
    size_t addr = address;

    while (readBytes < bytesCount) {
        readLength = bytesCount - readBytes < MAX_BUF_SIZE
                         ? bytesCount - readBytes
                         : MAX_BUF_SIZE;

        uint32_t *packet_ptr = reinterpret_cast<uint32_t *>(packet);
        packet_ptr[0] = addr;
        packet[4] = MAX_BUF_SIZE;

        // This set property will send the EEPROM address to be read
        CLEAR(cq);
        cq.query = UVC_SET_CUR; // bRequest
        cq.data = static_cast<unsigned char *>(packet);
        cq.size = MAX_BUF_SIZE; // MAX_BUF_SIZE;
        cq.unit = 0x03;         // wIndex
        cq.selector = 5;        // WValue for EEPROM register reads

        if (-1 == xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
            LOG(WARNING) << "Error in sending address to device, error: "
                         << errno << "(" << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }

        // This get property will get the value read from EEPROM address
        CLEAR(cq);
        cq.query = UVC_GET_CUR; // bRequest
        cq.data = static_cast<unsigned char *>(packet);
        cq.size = MAX_BUF_SIZE; // MAX_BUF_SIZE;
        cq.unit = 0x03;         // wIndex
        cq.selector = 5;        // WValue for EEPROM register reads

        if (-1 == xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
            LOG(WARNING) << "Error in reading data from device, error: "
                         << errno << "(" << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }

        memcpy(&data[readBytes], packet, readLength);
        readBytes += readLength;
        addr += readLength;
    }

    return Status::OK;
}

Status UsbEeprom::write(const uint32_t address, const uint8_t *data,
                        const size_t bytesCount) {
    struct uvc_xu_control_query cq;
    uint8_t packet[MAX_BUF_SIZE];
    size_t writeLen = 0;
    size_t writtenBytes = 0;
    uint32_t crtAddress = address;

    while (writtenBytes < bytesCount) {
        writeLen = bytesCount - writtenBytes > MAX_BUF_SIZE - 5
                       ? MAX_BUF_SIZE - 5
                       : bytesCount - writtenBytes;

        uint32_t *packet_ptr = reinterpret_cast<uint32_t *>(packet);
        packet_ptr[0] = crtAddress;
        packet[4] = writeLen;
        memcpy(&packet[5], data + writtenBytes, writeLen);

        // This set property will send the EEPROM address and data to be written
        // at the address
        CLEAR(cq);
        cq.query = UVC_SET_CUR; // bRequest
        cq.data = static_cast<unsigned char *>(packet);
        cq.size = MAX_BUF_SIZE; // MAX_BUF_SIZE;
        cq.unit = 0x03;         // wIndex
        cq.selector = 6;        // WValue for EEPROM register writes

        if (-1 == xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
            LOG(WARNING) << "Error in sending address to device, error: "
                         << errno << "(" << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }
        writtenBytes += writeLen;
        crtAddress += writeLen;
    }

    return Status::OK;
}

Status UsbEeprom::close() {
    m_implData->fd = -1;
    m_implData->name.clear();
    m_implData->driverPath.clear();

    return Status::OK;
}
