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
#include "usb_device.h"
#include "utils.h"
#include "utils_linux.h"

#include "device_utils.h"

#include <cmath>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <unordered_map>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define MAX_PACKET_SIZE (58)
#define MAX_BUF_SIZE (MAX_PACKET_SIZE + 2)

struct buffer {
    void *start;
    size_t length;
};

struct UsbDevice::ImplData {
    int fd;
    struct buffer *buffers;
    unsigned int buffersCount;
    struct v4l2_format fmt;
    bool started;
};

UsbDevice::UsbDevice(const aditof::DeviceConstructionData &data)
    : m_devData(data), m_implData(new UsbDevice::ImplData) {
    m_implData->fd = 0;
    m_implData->started = false;
    m_implData->buffers = nullptr;
    m_implData->buffersCount = 0;
    m_deviceDetails.sensorType = aditof::SensorType::SENSOR_96TOF1;
}

UsbDevice::~UsbDevice() {
    if (m_implData->started) {
        stop();
    }

    for (unsigned int i = 0; i < m_implData->buffersCount; ++i) {
        if (-1 == munmap(m_implData->buffers[i].start,
                         m_implData->buffers[i].length)) {
            LOG(WARNING) << "munmap, error:" << errno << "(" << strerror(errno)
                         << ")";
            return;
        }
    }
    if (m_implData->buffers)
        free(m_implData->buffers);

    if (m_implData->fd != 0) {
        if (-1 == close(m_implData->fd))
            LOG(WARNING) << "close, error:" << errno << "(" << strerror(errno)
                         << ")";
    }
}

aditof::Status UsbDevice::open() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    m_implData->fd =
        ::open(m_devData.driverPath.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (-1 == m_implData->fd) {
        LOG(WARNING) << "Cannot open '" << m_devData.driverPath
                     << "' error: " << errno << "(" << strerror(errno) << ")";
        return Status::UNREACHABLE;
    }

    CLEAR(m_implData->fmt);

    m_implData->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 == xioctl(m_implData->fd, VIDIOC_G_FMT, &m_implData->fmt)) {
        LOG(WARNING) << "VIDIOC_G_FMT, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDevice::start() {
    using namespace aditof;

    return Status::OK;
}

aditof::Status UsbDevice::stop() {
    using namespace aditof;

    if (!m_implData->started) {
        LOG(INFO) << "Device already stopped";
        return Status::BUSY;
    }
    LOG(INFO) << "Stopping device";

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(m_implData->fd, VIDIOC_STREAMOFF, &type)) {
        LOG(WARNING) << "VIDIOC_STREAMOFF, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    m_implData->started = false;

    return Status::OK;
}

aditof::Status
UsbDevice::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}

aditof::Status UsbDevice::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;

    Status status = Status::OK;

    // Buggy driver paranoia.
    unsigned int min;

    m_implData->fmt.fmt.pix.width = details.width;
    m_implData->fmt.fmt.pix.height = details.height;

    min = m_implData->fmt.fmt.pix.width * 2;
    if (m_implData->fmt.fmt.pix.bytesperline < min)
        m_implData->fmt.fmt.pix.bytesperline = min;
    min = m_implData->fmt.fmt.pix.bytesperline * m_implData->fmt.fmt.pix.height;
    if (m_implData->fmt.fmt.pix.sizeimage < min)
        m_implData->fmt.fmt.pix.sizeimage = min;

    if (-1 == xioctl(m_implData->fd, VIDIOC_S_FMT, &m_implData->fmt)) {
        LOG(WARNING) << "Failed to set Pixel Format, error: " << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_requestbuffers req;

    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(m_implData->fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            LOG(WARNING) << m_devData.driverPath
                         << " does not support memmory mapping";
        } else {
            LOG(WARNING) << "VIDIOC_REQBUFS, error:" << errno << "("
                         << strerror(errno) << ")";
        }
        return Status::GENERIC_ERROR;
    }

    if (req.count < 2) {
        LOG(WARNING) << "Insufficient buffer memory on "
                     << m_devData.driverPath;
        return Status::GENERIC_ERROR;
    }

    if (!m_implData->buffers) {
        m_implData->buffers =
            static_cast<buffer *>(calloc(req.count, sizeof(struct buffer)));
    }

    if (!m_implData->buffers) {
        LOG(WARNING) << "Out of memory";
        return Status::GENERIC_ERROR;
    }

    for (m_implData->buffersCount = 0; m_implData->buffersCount < req.count;
         ++m_implData->buffersCount) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = m_implData->buffersCount;

        if (-1 == xioctl(m_implData->fd, VIDIOC_QUERYBUF, &buf)) {
            LOG(WARNING) << "VIDIOC_QUERYBUF, error:" << errno << "("
                         << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }

        // TO DO: Check if is better to use mremap()

        m_implData->buffers[m_implData->buffersCount].length = buf.length;
        m_implData->buffers[m_implData->buffersCount].start =
            mmap(nullptr, // start anywhere ,
                 buf.length,
                 PROT_READ | PROT_WRITE, // required,
                 MAP_SHARED,             // recommended ,
                 m_implData->fd, buf.m.offset);

        if (MAP_FAILED == m_implData->buffers[m_implData->buffersCount].start) {
            LOG(WARNING) << "mmap, error:" << errno << "(" << strerror(errno)
                         << ")";
            return Status::GENERIC_ERROR;
        }
    }

    for (unsigned int i = 0; i < m_implData->buffersCount; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(m_implData->fd, VIDIOC_QBUF, &buf)) {
            LOG(WARNING) << "VIDIOC_QBUF, error:" << errno << "("
                         << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}

aditof::Status UsbDevice::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;

    return Status::OK;
}

aditof::Status UsbDevice::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}

aditof::Status UsbDevice::readEeprom(uint32_t address, uint8_t *data,
                                     size_t length) {
    using namespace aditof;

    struct uvc_xu_control_query cq;
    uint8_t packet[MAX_BUF_SIZE];
    size_t readBytes = 0;
    size_t readLength = 0;
    size_t addr = address;

    while (readBytes < length) {
        readLength = length - readBytes < MAX_BUF_SIZE ? length - readBytes
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

aditof::Status UsbDevice::writeEeprom(uint32_t address, const uint8_t *data,
                                      size_t length) {
    using namespace aditof;

    struct uvc_xu_control_query cq;
    uint8_t packet[MAX_BUF_SIZE];
    size_t writeLen = 0;
    size_t writtenBytes = 0;

    while (writtenBytes < length) {
        writeLen = length - writtenBytes > MAX_BUF_SIZE - 5
                       ? MAX_BUF_SIZE - 5
                       : length - writtenBytes;

        uint32_t *packet_ptr = reinterpret_cast<uint32_t *>(packet);
        packet_ptr[0] = address;
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
        address += writeLen;
    }

    return Status::OK;
}

aditof::Status UsbDevice::readAfeTemp(float &temperature) {
    using namespace aditof;

    struct uvc_xu_control_query cq;
    float buffer[2];

    // This get property will get the value from temperature sensor
    CLEAR(cq);
    cq.query = UVC_GET_CUR; // bRequest
    cq.data = reinterpret_cast<unsigned char *>(buffer);
    cq.size = 8;     // MAX_BUF_SIZE;
    cq.unit = 0x03;  // wIndex
    cq.selector = 3; // WValue for TempSensor register reads
    if (-1 == xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
        LOG(WARNING) << "Error in reading data from device, error: " << errno
                     << "(" << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    temperature = buffer[0];

    return Status::OK;
}

aditof::Status UsbDevice::readLaserTemp(float &temperature) {
    using namespace aditof;

    struct uvc_xu_control_query cq;
    float buffer[2];

    // This get property will get the value from temperature sensor
    CLEAR(cq);
    cq.query = UVC_GET_CUR; // bRequest
    cq.data = reinterpret_cast<unsigned char *>(buffer);
    cq.size = 8;     // MAX_BUF_SIZE;
    cq.unit = 0x03;  // wIndex
    cq.selector = 3; // WValue for TempSensor register reads
    if (-1 == xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
        LOG(WARNING) << "Error in reading data from device, error: " << errno
                     << "(" << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    temperature = buffer[1];

    return Status::OK;
}

aditof::Status UsbDevice::getDetails(aditof::DeviceDetails &details) const {
    details = m_deviceDetails;
    return aditof::Status::OK;
}
