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
#include "connections/usb/usb_depth_sensor.h"
#include "usb_linux_utils.h"
#include "utils.h"

#include "device_utils.h"

#include <cmath>
#include <fcntl.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#include <cstring>
#include <unistd.h>
#endif
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <unordered_map>

struct buffer {
    void *start;
    size_t length;
};

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};

struct UsbDepthSensor::ImplData {
    int fd;
    struct buffer *buffers;
    unsigned int buffersCount;
    struct v4l2_format fmt;
    bool opened;
    bool started;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
};

UsbDepthSensor::UsbDepthSensor(const std::string &driverPath)
    : m_driverPath(driverPath), m_implData(new UsbDepthSensor::ImplData) {
    m_implData->fd = 0;
    m_implData->opened = false;
    m_implData->started = false;
    m_implData->buffers = nullptr;
    m_implData->buffersCount = 0;
    m_sensorDetails.connectionType = aditof::ConnectionType::USB;
}

UsbDepthSensor::~UsbDepthSensor() {
    if (m_implData->started) {
        stop();
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
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

aditof::Status UsbDepthSensor::open() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    m_implData->fd = ::open(m_driverPath.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (-1 == m_implData->fd) {
        LOG(WARNING) << "Cannot open '" << m_driverPath << "' error: " << errno
                     << "(" << strerror(errno) << ")";
        return Status::UNREACHABLE;
    }

    CLEAR(m_implData->fmt);

    m_implData->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 ==
        UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_G_FMT, &m_implData->fmt)) {
        LOG(WARNING) << "VIDIOC_G_FMT, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    m_implData->opened = true;
    m_implData->started = true;
    return status;
}

aditof::Status UsbDepthSensor::start() {
    using namespace aditof;

    if (m_implData->started) {
        return Status::OK;
    }
    LOG(INFO) << "Starting device";
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_STREAMON, &type)) {
        LOG(WARNING) << "VIDIOC_STREAMON, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }
    m_implData->started = true;
    return Status::OK;
}

aditof::Status UsbDepthSensor::stop() {
    using namespace aditof;

    if (!m_implData->started) {
        LOG(INFO) << "Device already stopped";
        return Status::BUSY;
    }
    LOG(INFO) << "Stopping device";

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_STREAMOFF, &type)) {
        LOG(WARNING) << "VIDIOC_STREAMOFF, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    m_implData->started = false;

    return Status::OK;
}

aditof::Status UsbDepthSensor::getAvailableFrameTypes(
    std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    // Hardcored for now
    FrameDetails details;

    details.width = aditof::USB_FRAME_WIDTH;
    details.height = aditof::USB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height * 2; //TODO
    details.type = "depth_ir";
    types.push_back(details);

    details.width = aditof::USB_FRAME_WIDTH;
    details.height = aditof::USB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height; //TODO
    details.type = "depth";
    types.push_back(details);

    details.width = aditof::USB_FRAME_WIDTH;
    details.height = aditof::USB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height; //TODO
    details.type = "ir";
    types.push_back(details);

    // TO DO: Should get these details from the hardware/firmware

    return status;
}

aditof::Status
UsbDepthSensor::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;

    Status status = Status::OK;

    // Buggy driver paranoia.
    unsigned int min;

    m_implData->fmt.fmt.pix.width = details.fullDataWidth;
    m_implData->fmt.fmt.pix.height = details.fullDataHeight;

    min = m_implData->fmt.fmt.pix.width * 2;
    if (m_implData->fmt.fmt.pix.bytesperline < min)
        m_implData->fmt.fmt.pix.bytesperline = min;
    min = m_implData->fmt.fmt.pix.bytesperline * m_implData->fmt.fmt.pix.height;
    if (m_implData->fmt.fmt.pix.sizeimage < min)
        m_implData->fmt.fmt.pix.sizeimage = min;

    if (-1 ==
        UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_S_FMT, &m_implData->fmt)) {
        LOG(WARNING) << "Failed to set Pixel Format, error: " << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_requestbuffers req;

    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            LOG(WARNING) << m_driverPath << " does not support memmory mapping";
        } else {
            LOG(WARNING) << "VIDIOC_REQBUFS, error:" << errno << "("
                         << strerror(errno) << ")";
        }
        return Status::GENERIC_ERROR;
    }

    if (req.count < 2) {
        LOG(WARNING) << "Insufficient buffer memory on " << m_driverPath;
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

        if (-1 ==
            UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_QUERYBUF, &buf)) {
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

        if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_QBUF, &buf)) {
            LOG(WARNING) << "VIDIOC_QBUF, error:" << errno << "("
                         << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}

aditof::Status UsbDepthSensor::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;

    if (!firmware) {
        LOG(WARNING) << "No firmware provided";
        return Status::INVALID_ARGUMENT;
    }

    struct uvc_xu_control_query cq;
    unsigned char buf[MAX_BUF_SIZE];
    size_t written_bytes = 0;
    __useconds_t sleepDuration =
        100000; /* Keep 100 ms delay between 'program' calls */

    while (written_bytes < size) {

        CLEAR(cq);
        cq.query = UVC_SET_CUR; // bRequest
        cq.unit = 0x03;         // wIndex of Extension Unit
        cq.selector = 1;        // WValue for AFE Programming
        cq.data = buf;
        cq.size = MAX_BUF_SIZE;

        usleep(5000);
        if ((size - written_bytes) > MAX_PACKET_SIZE) {
            buf[0] = 0x01;
            buf[1] = MAX_PACKET_SIZE;
            memcpy(&buf[2], firmware + written_bytes, MAX_PACKET_SIZE);

            if (-1 ==
                UsbLinuxUtils::xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
                LOG(WARNING)
                    << "Programming AFE error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
            written_bytes += MAX_PACKET_SIZE;
        } else {
            CLEAR(buf);
            buf[0] = 0x02;
            buf[1] = static_cast<unsigned char>(size - written_bytes);
            memcpy(&buf[2], firmware + written_bytes, buf[1]);

            cq.data = buf;
            if (-1 ==
                UsbLinuxUtils::xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
                LOG(WARNING)
                    << "Programming AFE error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
            written_bytes = size;
        }
    }

    // TO DO: Check if it is really neccessary or if the delay is not to much
    usleep(sleepDuration);

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_STREAMON, &type)) {
        LOG(WARNING) << "VIDIOC_STREAMON, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    m_implData->started = true;

    return Status::OK;
}

aditof::Status UsbDepthSensor::getFrame(uint16_t *buffer,
                                        aditof::BufferInfo *bufferInfo) {
    using namespace aditof;
    Status status = Status::OK;

    if (!buffer) {
        LOG(WARNING) << "Invalid adddress to buffer provided";
        return Status::INVALID_ARGUMENT;
    }

    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(m_implData->fd, &fds);

    // Timeout : Ensure this compensates for max delays added for programming
    // cycle defined in 'Device::program'
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(m_implData->fd + 1, &fds, nullptr, nullptr, &tv);

    if (-1 == r) {
        if (EINTR == errno) {
            LOG(WARNING) << "select, error: " << errno << "(" << strerror(errno)
                         << ")";
            return Status::GENERIC_ERROR;
        }
    }

    if (0 == r) {
        LOG(WARNING) << "select timeout: ";
        return Status::BUSY;
    }

    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_DQBUF, &buf)) {
        LOG(WARNING) << "Stream Error";
        switch (errno) {
        case EAGAIN:
            break;

        case EIO:
            // Could ignore EIO, see spec.
            // fall through

        default: {
            LOG(WARNING) << "VIDIOC_DQBUF, error: " << errno << "("
                         << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }
        }
    }

    if (buf.index >= m_implData->buffersCount) {
        LOG(WARNING) << "buffer index out of range";
        return Status::INVALID_ARGUMENT;
    }

    unsigned int width = m_implData->fmt.fmt.pix.width;
    unsigned int height = m_implData->fmt.fmt.pix.height;
    const char *pdata =
        static_cast<const char *>(m_implData->buffers[buf.index].start);

    aditof::deinterleave(pdata, buffer, height * width * 3 / 2, width, height);

    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_QBUF, &buf)) {
        LOG(WARNING) << "VIDIOC_QBUF, error: " << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDepthSensor::readAfeRegisters(const uint16_t *address,
                                                uint16_t *data, size_t length) {
    using namespace aditof;
    int ret;

    if (address == nullptr) {
        LOG(ERROR) << "Received AfeRegisters address null pointer";
        return Status::INVALID_ARGUMENT;
    }

    if (data == nullptr) {
        LOG(ERROR) << "Received AfeRegisters data null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(length > 0);

    for (size_t i = 0; i < length; ++i) {
        ret = UsbLinuxUtils::uvcExUnitReadOnePacket(
            m_implData->fd, 2, reinterpret_cast<uint8_t *>(address[i]), 0,
            reinterpret_cast<uint8_t *>(&data[i]), 2, 2);
        if (ret < 0) {
            LOG(WARNING)
                << "Failed to read a packet via UVC extension unit. Error: "
                << ret;
            return Status::GENERIC_ERROR;
        }
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::writeAfeRegisters(const uint16_t *address,
                                                 const uint16_t *data,
                                                 size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    if (address == nullptr) {
        LOG(ERROR) << "Received AfeRegisters address null pointer";
        return Status::INVALID_ARGUMENT;
    }

    if (data == nullptr) {
        LOG(ERROR) << "Received AfeRegisters data null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(length > 0);

    struct uvc_xu_control_query cq;
    unsigned char buf[MAX_BUF_SIZE];

    CLEAR(cq);
    cq.query = UVC_SET_CUR; // bRequest
    cq.unit = 0x03;         // wIndex of Extension Unit
    cq.selector = 1;        // WValue for AFE Programming
    cq.data = buf;
    cq.size = MAX_BUF_SIZE;

    size_t elementCnt = 0;
    const uint8_t regSize =
        sizeof(address[0]); // Size (in bytes) of an AFE register

    length *= 2 * sizeof(uint16_t);

    const uint8_t *pAddr = reinterpret_cast<const uint8_t *>(address);
    const uint8_t *pData = reinterpret_cast<const uint8_t *>(data);
    const uint8_t *ptr = pAddr;
    bool pointingAtAddr = true;

    while (length) {
        memset(buf, 0, MAX_BUF_SIZE);
        buf[0] = length > MAX_PACKET_SIZE ? 0x01 : 0x02;
        buf[1] = length > MAX_PACKET_SIZE ? MAX_PACKET_SIZE : length;

        for (int i = 0; i < buf[1]; ++i) {
            // once every two bytes (or more if reg size is bigger) the ptr is switch to point to the
            // other array (data or addr) in order to interleave the values to be send */
            if ((elementCnt / regSize) && (elementCnt % regSize == 0)) {
                if (pointingAtAddr) {
                    pAddr = ptr;
                    ptr = pData;
                } else {
                    pData = ptr;
                    ptr = pAddr;
                }
                pointingAtAddr = !pointingAtAddr;
            }
            buf[2 + i] = *ptr++;
            ++elementCnt;
        }

        length -= buf[1];

        if (-1 ==
            UsbLinuxUtils::xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno);
        }
    }

    return status;
}

aditof::Status
UsbDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::getHandle(void **handle) {
    if (m_implData->opened) {
        *handle = &m_implData->fd;
        return aditof::Status::OK;
    } else {
        *handle = nullptr;
        LOG(ERROR) << "Won't return the handle. Device hasn't been opened yet.";
        return aditof::Status::UNAVAILABLE;
    }
}

aditof::Status UsbDepthSensor::getName(std::string &sensorName) const {
    sensorName = m_sensorName;

    return aditof::Status::OK;
}
