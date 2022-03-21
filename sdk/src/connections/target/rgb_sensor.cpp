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
#include "rgb_sensor.h"
#include "aditof/frame_operations.h"
#include "utils.h"

#include <algorithm>
#include <arm_neon.h>
#include <assert.h>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#include <cstring>
#include <unistd.h>
#endif
#include <iostream>
#include <linux/videodev2.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unordered_map>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define V4L2_CID_AD_DEV_SET_CHIP_CONFIG 0xA00A00
#define V4L2_CID_AD_DEV_READ_REG 0xA00A01
#define CTRL_PACKET_SIZE 4096

struct buffer {
    void *start;
    size_t length;
};

struct VideoDev {
    int fd;
    int sfd;
    struct buffer *videoBuffers;
    unsigned int nVideoBuffers;
    struct v4l2_plane planes[8];
    enum v4l2_buf_type videoBuffersType;
    bool started;

    VideoDev()
        : fd(-1), sfd(-1), videoBuffers(nullptr), nVideoBuffers(0),
          started(false) {}
};

struct RgbSensor::ImplData {
    uint8_t numVideoDevs;
    struct VideoDev *videoDevs;
    aditof::FrameDetails frameDetails;
    ImplData()
        : numVideoDevs(1),
          videoDevs(nullptr), frameDetails{0, 0, 0, 0, 0, 0, ""} {}
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

RgbSensor::RgbSensor(const std::string &driverPath,
                     const std::string &driverSubPath,
                     const std::string &captureDev)
    : m_driverPath(driverPath), m_driverSubPath(driverSubPath),
      m_captureDev(captureDev), m_implData(new RgbSensor::ImplData) {
    m_sensorName = "ov2735";
}

RgbSensor::~RgbSensor() {
    struct VideoDev *dev;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev && dev->started) {
            stop();
        }
    }

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (!dev) {
            continue;
        }
        for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
            if (munmap(dev->videoBuffers[i].start,
                       dev->videoBuffers[i].length) == -1) {
                LOG(WARNING)
                    << "munmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
        }
        free(dev->videoBuffers);

        if (close(dev->fd) == -1) {
            LOG(WARNING) << "close m_implData->fd error "
                         << "errno: " << errno << " error: " << strerror(errno);
        }
    }
}

aditof::Status RgbSensor::open() {
    using namespace aditof;
    Status status = Status::OK;
    LOG(INFO) << "Opening device";

    struct stat st;
    struct v4l2_capability cap;
    struct VideoDev *dev;

    const char *devName;

    std::vector<std::string> driverPaths;
    Utils::splitIntoTokens(m_driverPath, '|', driverPaths);

    std::vector<std::string> driverSubPaths;
    Utils::splitIntoTokens(m_driverSubPath, '|', driverSubPaths);

    std::vector<std::string> cards;
    std::string captureDeviceName(m_captureDev);
    Utils::splitIntoTokens(captureDeviceName, '|', cards);

    LOG(INFO) << "Looking for the following cards:";
    for (const auto card : cards) {
        LOG(INFO) << card;
    }

    m_implData->numVideoDevs = driverSubPaths.size();

    assert(m_implData->numVideoDevs > 0);
    m_implData->videoDevs = new VideoDev[m_implData->numVideoDevs];

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        devName = driverPaths.at(i).c_str();
        dev = &m_implData->videoDevs[i];

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

        dev->fd = ::open(devName, O_RDWR | O_NONBLOCK, 0);
        if (dev->fd == -1) {
            LOG(WARNING) << "Cannot open " << devName << "errno: " << errno
                         << "error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        if (xioctl(dev->fd, VIDIOC_QUERYCAP, &cap) == -1) {
            LOG(WARNING) << devName << " VIDIOC_QUERYCAP error";
            return Status::GENERIC_ERROR;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            LOG(WARNING) << devName << " device is not able to capture";
            return Status::GENERIC_ERROR;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            LOG(WARNING) << devName << " device is not able to stream";
            return Status::GENERIC_ERROR;
        }

        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;
        CLEAR(cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl(dev->fd, VIDIOC_CROPCAP, &cropcap)) {
            crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            crop.c = cropcap.defrect; /* reset to default */

            if (-1 == xioctl(dev->fd, VIDIOC_S_CROP, &crop)) {
                switch (errno) {
                case EINVAL:
                    break;
                default:
                    break;
                }
            }
        } else {
            /* Error ignored */
        }

        /* Set the frame format in the driver */
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = RGB_FRAME_WIDTH;
        fmt.fmt.pix.height = RGB_FRAME_HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB10;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (xioctl(dev->fd, VIDIOC_S_FMT, &fmt) == -1) {
            LOG(WARNING) << "Setting Pixel Format error, errno: " << errno
                         << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;
    }

    return status;
}

aditof::Status RgbSensor::start() {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;
    struct v4l2_buffer buf;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev->started) {
            LOG(INFO) << "Device already started";
            return Status::BUSY;
        }
        LOG(INFO) << "Starting device " << i;

        for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
                LOG(WARNING)
                    << "mmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }
        }
        dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(dev->fd, VIDIOC_STREAMON, &dev->videoBuffersType) != 0) {
            LOG(WARNING) << "VIDIOC_STREAMON error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        dev->started = true;
    }
    return status;
}

aditof::Status RgbSensor::stop() {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;
    enum v4l2_buf_type type;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        if (!dev->started) {
            LOG(INFO) << "Device " << i << " already stopped";
            return Status::BUSY;
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(dev->fd, VIDIOC_STREAMOFF, &type))
            return Status::GENERIC_ERROR;
    }

    dev->started = false;
    return status;
}

aditof::Status
RgbSensor::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    FrameDetails details;

    details.rgbWidth = aditof::RGB_FRAME_WIDTH;
    details.rgbHeight = aditof::RGB_FRAME_HEIGHT;
    details.type = "rgb";
    types.push_back(details);

    return status;
}

aditof::Status RgbSensor::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (details.type != m_implData->frameDetails.type) {
            for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
                if (munmap(dev->videoBuffers[i].start,
                           dev->videoBuffers[i].length) == -1) {
                    LOG(WARNING)
                        << "munmap error "
                        << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }
            }
            free(dev->videoBuffers);
            dev->nVideoBuffers = 0;
        } else if (dev->nVideoBuffers) {
            return status;
        }

        /* Allocate the video buffers in the driver */
        CLEAR(req);
        req.count = 2;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) == -1) {
            LOG(WARNING) << "VIDIOC_REQBUFS error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        dev->videoBuffers =
            (buffer *)calloc(req.count, sizeof(*dev->videoBuffers));
        if (!dev->videoBuffers) {
            LOG(WARNING) << "Failed to allocate video m_implData->videoBuffers";
            return Status::GENERIC_ERROR;
        }

        for (dev->nVideoBuffers = 0; dev->nVideoBuffers < req.count;
             dev->nVideoBuffers++) {
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = dev->nVideoBuffers;

            if (xioctl(dev->fd, VIDIOC_QUERYBUF, &buf) == -1) {
                LOG(WARNING)
                    << "VIDIOC_QUERYBUF error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            dev->videoBuffers[dev->nVideoBuffers].length = buf.length;
            dev->videoBuffers[dev->nVideoBuffers].start =
                mmap(NULL /* start anywhere */, buf.length,
                     PROT_READ | PROT_WRITE /* required */,
                     MAP_SHARED /* recommended */, dev->fd, buf.m.offset);

            if (MAP_FAILED == dev->videoBuffers[dev->nVideoBuffers].start)
                return Status::GENERIC_ERROR;
        }
    }

    m_implData->frameDetails = details;

    return status;
}

aditof::Status RgbSensor::program(const uint8_t *firmware, size_t size) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status RgbSensor::getFrame(uint16_t *buffer,
                                   aditof::BufferInfo *bufferInfo) {
    using namespace aditof;
    struct v4l2_buffer buf[m_implData->numVideoDevs];
    struct VideoDev *dev;
    Status status;

    if (buffer == nullptr) {
        LOG(ERROR) << "Received buffer null pointer";
        return Status::INVALID_ARGUMENT;
    }

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        status = waitForBufferPrivate(dev);
        if (status != Status::OK) {
            return status;
        }

        status = dequeueInternalBufferPrivate(buf[i], dev);
        if (status != Status::OK) {
            return status;
        }
        unsigned int buf_data_len;
        uint8_t *pdata[m_implData->numVideoDevs];
        status = getInternalBufferPrivate(&pdata[i], buf_data_len, buf[i], dev);
        if (status != Status::OK) {
            return status;
        }

        memcpy(buffer, pdata[i], RGB_FRAME_HEIGHT * RGB_FRAME_WIDTH * 2);

        status = enqueueInternalBufferPrivate(buf[i], dev);
        if (status != Status::OK) {
            return status;
        }
    }
    bufferInfo->timestamp =
        buf[0].timestamp.tv_sec * 1000000 + buf[0].timestamp.tv_usec;
    return status;
}

aditof::Status RgbSensor::readAfeRegisters(const uint16_t *address,
                                           uint16_t *data, size_t length) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status RgbSensor::writeAfeRegisters(const uint16_t *address,
                                            const uint16_t *data,
                                            size_t length) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status RgbSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status RgbSensor::getHandle(void **handle) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status RgbSensor::getName(std::string &sensorName) const {
    sensorName = m_sensorName;

    return aditof::Status::OK;
}

aditof::Status RgbSensor::waitForBufferPrivate(struct VideoDev *dev) {
    fd_set fds;
    struct timeval tv;
    int r;

    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    tv.tv_sec = 20;
    tv.tv_usec = 0;

    r = select(dev->fd + 1, &fds, NULL, NULL, &tv);

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

aditof::Status RgbSensor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                       struct VideoDev *dev) {
    using namespace aditof;
    Status status = Status::OK;

    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(dev->fd, VIDIOC_DQBUF, &buf) == -1) {
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

    if (buf.index >= dev->nVideoBuffers) {
        LOG(WARNING) << "Not enough buffers avaialable";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status
RgbSensor::getInternalBufferPrivate(uint8_t **buffer, uint32_t &buf_data_len,
                                    const struct v4l2_buffer &buf,
                                    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = m_implData->frameDetails.rgbWidth *
                   m_implData->frameDetails.rgbHeight * 2;

    return aditof::Status::OK;
}

aditof::Status RgbSensor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                       struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

aditof::Status RgbSensor::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    if (dev->fd != -1) {
        fileDescriptor = dev->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}

aditof::Status RgbSensor::waitForBuffer() { return waitForBufferPrivate(); }

aditof::Status RgbSensor::dequeueInternalBuffer(struct v4l2_buffer &buf) {
    return dequeueInternalBufferPrivate(buf);
}

aditof::Status RgbSensor::getInternalBuffer(uint8_t **buffer,
                                            uint32_t &buf_data_len,
                                            const struct v4l2_buffer &buf) {
    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

aditof::Status RgbSensor::enqueueInternalBuffer(struct v4l2_buffer &buf) {
    return enqueueInternalBufferPrivate(buf);
}
