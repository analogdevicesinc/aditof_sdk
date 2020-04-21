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
#include "local_device.h"
#include "target_definitions.h"
#include "utils.h"
#include <aditof/frame_operations.h>
#include <fstream>

#include <algorithm>
#include <arm_neon.h>
#include <cmath>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/videodev2.h>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <target_definitions.h>
#include <unordered_map>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

struct buffer {
    void *start;
    size_t length;
};

struct LocalDevice::ImplData {
    int fd;
    int sfd;
    struct buffer *videoBuffers;
    unsigned int nVideoBuffers;
    struct v4l2_plane planes[8];
    aditof::FrameDetails frameDetails;
    bool started;
    enum v4l2_buf_type videoBuffersType;

    ImplData()
        : fd(-1), sfd(-1), videoBuffers(nullptr),
          nVideoBuffers(0), frameDetails{0, 0, "", {0.0f, 1.0f}},
          started(false) {}
};

// TO DO: This exists in linux_utils.h which is not included on Dragoboard.
// Should not have duplicated code if possible.
static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

LocalDevice::LocalDevice(const aditof::DeviceConstructionData &data)
    : m_devData(data), m_implData(new LocalDevice::ImplData) {

    m_deviceDetails.sensorType = aditof::SensorType::SENSOR_96TOF1;
}

LocalDevice::~LocalDevice() {
    if (m_implData->started) {
        stop();
    }

    for (unsigned int i = 0; i < m_implData->nVideoBuffers; i++) {
        if (munmap(m_implData->videoBuffers[i].start,
                   m_implData->videoBuffers[i].length) == -1) {
            LOG(WARNING) << "munmap error "
                         << "errno: " << errno << " error: " << strerror(errno);
        }
    }
    free(m_implData->videoBuffers);

    if (close(m_implData->fd) == -1) {
        LOG(WARNING) << "close m_implData->fd error "
                     << "errno: " << errno << " error: " << strerror(errno);
    }

    if (close(m_implData->sfd) == -1) {
        LOG(WARNING) << "close m_implData->sfd error "
                     << "errno: " << errno << " error: " << strerror(errno);
    }
}

aditof::Status LocalDevice::open() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    struct stat st;
    struct v4l2_capability cap;

    std::vector<std::string> paths;
    std::stringstream ss(m_devData.driverPath);
    std::string token;
    while (std::getline(ss, token, ';')) {
        paths.push_back(token);
    }

    const char *devName = paths.front().c_str();
    const char *subDevName = paths.back().c_str();

    /* Open V4L2 device */
    if (stat(devName, &st) == -1) {
        LOG(WARNING) << "Cannot identify " << devName << "errno: " << errno
                     << "error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (!S_ISCHR(st.st_mode)) {
        LOG(WARNING) << devName << " is not a valid device";
        return Status::GENERIC_ERROR;
    }

    m_implData->fd = ::open(devName, O_RDWR | O_NONBLOCK, 0);
    if (m_implData->fd == -1) {
        LOG(WARNING) << "Cannot open " << devName << "errno: " << errno
                     << "error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (xioctl(m_implData->fd, VIDIOC_QUERYCAP, &cap) == -1) {
        LOG(WARNING) << devName << " VIDIOC_QUERYCAP error";
        return Status::GENERIC_ERROR;
    }

    if (strcmp((char *)cap.card, CAPTURE_DEVICE_NAME)) {
        LOG(WARNING) << "CAPTURE Device " << cap.card;
        return Status::GENERIC_ERROR;
    }

    if (!(cap.capabilities &
          (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
        LOG(WARNING) << devName << " is not a video capture device";
        return Status::GENERIC_ERROR;
    }

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        m_implData->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else {
        m_implData->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        LOG(WARNING) << devName << " does not support streaming i/o";
        return Status::GENERIC_ERROR;
    }

    /* Open V4L2 subdevice */
    if (stat(subDevName, &st) == -1) {
        LOG(WARNING) << "Cannot identify " << subDevName << " errno: " << errno
                     << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (!S_ISCHR(st.st_mode)) {
        LOG(WARNING) << subDevName << " is not a valid device";
        return Status::GENERIC_ERROR;
    }

    m_implData->sfd = ::open(subDevName, O_RDWR | O_NONBLOCK, 0);
    if (m_implData->sfd == -1) {
        LOG(WARNING) << "Cannot open " << subDevName << " errno: " << errno
                     << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status LocalDevice::start() {
    using namespace aditof;
    Status status = Status::OK;

    if (m_implData->started) {
        LOG(INFO) << "Device already started";
        return Status::BUSY;
    }
    LOG(INFO) << "Starting device";

    struct v4l2_buffer buf;
    for (unsigned int i = 0; i < m_implData->nVideoBuffers; i++) {
        CLEAR(buf);
        buf.type = m_implData->videoBuffersType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = m_implData->planes;
        buf.length = 1;

        if (xioctl(m_implData->fd, VIDIOC_QBUF, &buf) == -1) {
            LOG(WARNING) << "mmap error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    }

    if (xioctl(m_implData->fd, VIDIOC_STREAMON,
               &m_implData->videoBuffersType) == -1) {
        LOG(WARNING) << "VIDIOC_STREAMON error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_implData->started = true;

    return status;
}

aditof::Status LocalDevice::stop() {
    using namespace aditof;
    Status status = Status::OK;

    if (!m_implData->started) {
        LOG(INFO) << "Device already stopped";
        return Status::BUSY;
    }
    LOG(INFO) << "Stopping device";

    if (xioctl(m_implData->fd, VIDIOC_STREAMOFF,
               &m_implData->videoBuffersType) == -1) {
        LOG(WARNING) << "VIDIOC_STREAMOFF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_implData->started = false;

    return status;
}

aditof::Status
LocalDevice::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}

aditof::Status LocalDevice::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    struct v4l2_requestbuffers req;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    size_t length, offset;

    if (details != m_implData->frameDetails) {
        for (unsigned int i = 0; i < m_implData->nVideoBuffers; i++) {
            if (munmap(m_implData->videoBuffers[i].start,
                       m_implData->videoBuffers[i].length) == -1) {
                LOG(WARNING)
                    << "munmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }
        }
        free(m_implData->videoBuffers);
        m_implData->nVideoBuffers = 0;
    } else if (m_implData->nVideoBuffers) {
        return status;
    }

    /* Set the frame format in the driver */
    CLEAR(fmt);
    fmt.type = m_implData->videoBuffersType;
    fmt.fmt.pix.width = details.width;
    fmt.fmt.pix.height = details.height;

    if (xioctl(m_implData->fd, VIDIOC_S_FMT, &fmt) == -1) {
        LOG(WARNING) << "Setting Pixel Format error, errno: " << errno
                     << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    /* Allocate the video buffers in the driver */
    CLEAR(req);
    req.count = 4;
    req.type = m_implData->videoBuffersType;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_implData->fd, VIDIOC_REQBUFS, &req) == -1) {
        LOG(WARNING) << "VIDIOC_REQBUFS error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_implData->videoBuffers =
        (buffer *)calloc(req.count, sizeof(*m_implData->videoBuffers));
    if (!m_implData->videoBuffers) {
        LOG(WARNING) << "Failed to allocate video m_implData->videoBuffers";
        return Status::GENERIC_ERROR;
    }

    for (m_implData->nVideoBuffers = 0; m_implData->nVideoBuffers < req.count;
         m_implData->nVideoBuffers++) {
        CLEAR(buf);
        buf.type = m_implData->videoBuffersType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = m_implData->nVideoBuffers;
        buf.m.planes = m_implData->planes;
        buf.length = 1;

        if (xioctl(m_implData->fd, VIDIOC_QUERYBUF, &buf) == -1) {
            LOG(WARNING) << "VIDIOC_QUERYBUF error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        if (m_implData->videoBuffersType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            length = buf.length;
            offset = buf.m.offset;
        } else {
            length = buf.m.planes[0].length;
            offset = buf.m.planes[0].m.mem_offset;
        }

        m_implData->videoBuffers[m_implData->nVideoBuffers].start =
            mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED,
                 m_implData->fd, offset);

        if (m_implData->videoBuffers[m_implData->nVideoBuffers].start ==
            MAP_FAILED) {
            LOG(WARNING) << "mmap error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        m_implData->videoBuffers[m_implData->nVideoBuffers].length = length;
    }

    m_implData->frameDetails = details;

    return status;
}

aditof::Status LocalDevice::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}

aditof::Status LocalDevice::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}

aditof::Status LocalDevice::readAfeRegisters(const uint16_t *address,
                                             uint16_t *data, size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}

aditof::Status LocalDevice::writeAfeRegisters(const uint16_t *address,
                                              const uint16_t *data,
                                              size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}

aditof::Status LocalDevice::getDetails(aditof::DeviceDetails &details) const {
    details = m_deviceDetails;
    return aditof::Status::OK;
}

aditof::Status LocalDevice::waitForBuffer() {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(m_implData->fd, &fds);

    tv.tv_sec = 4;
    tv.tv_usec = 0;

    r = select(m_implData->fd + 1, &fds, NULL, NULL, &tv);

    if (r == -1) {
        LOG(WARNING) << "select error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    } else if (r == 0) {
        LOG(WARNING) << "select timeout";
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof ::Status::OK;
}

aditof::Status LocalDevice::dequeueInternalBuffer(struct v4l2_buffer &buf) {
    using namespace aditof;
    Status status = Status::OK;

    CLEAR(buf);
    buf.type = m_implData->videoBuffersType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = m_implData->planes;

    if (xioctl(m_implData->fd, VIDIOC_DQBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_DQBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        switch (errno) {
        case EAGAIN:
        case EIO:
            break;
        default:
            return Status::GENERIC_ERROR;
        }
    }

    if (buf.index >= m_implData->nVideoBuffers) {
        LOG(WARNING) << "Not enough buffers avaialable";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status LocalDevice::getInternalBuffer(uint8_t **buffer,
                                              uint32_t &buf_data_len,
                                              const struct v4l2_buffer &buf) {

    *buffer = static_cast<uint8_t *>(m_implData->videoBuffers[buf.index].start);
    buf_data_len = m_implData->frameDetails.width *
                   m_implData->frameDetails.height * 3 / 2;

    return aditof::Status::OK;
}

aditof::Status LocalDevice::enqueueInternalBuffer(struct v4l2_buffer &buf) {
    if (xioctl(m_implData->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

aditof::Status LocalDevice::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;

    if (m_implData->fd != -1) {
        fileDescriptor = m_implData->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}
