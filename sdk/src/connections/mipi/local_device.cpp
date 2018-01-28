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
#include <aditof/frame_operations.h>
#include <fstream>

extern "C" {
#include "eeprom.h"
#include "temp_sensor.h"
}

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

#define V4L2_CID_AD_DEV_SET_CHIP_CONFIG 0xA00A00
#define V4L2_CID_AD_DEV_READ_REG 0xA00A01
#define CTRL_PACKET_SIZE 4096

#define LASER_TEMP_SENSOR_I2C_ADDR 0x49
#define AFE_TEMP_SENSOR_I2C_ADDR 0x4b

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

struct LocalDevice::ImplData {
    struct VideoDev *videoDevs;
    aditof::FrameDetails frameDetails;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
    eeprom edev;

    ImplData()
        :
          videoDevs(nullptr), frameDetails{0, 0, ""} {}
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
    m_implData->calibration_cache =
        std::unordered_map<std::string, CalibrationData>();

#if defined(CHICONY_006)
    m_deviceDetails.sensorType = aditof::SensorType::SENSOR_CHICONY;
#elif defined(FX1)
    m_deviceDetails.sensorType = aditof::SensorType::SENSOR_FX1;
#else
    m_deviceDetails.sensorType = aditof::SensorType::SENSOR_96TOF1;
#endif
}

LocalDevice::~LocalDevice() {
    struct VideoDev *dev;

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev->started) {
             stop();
        }
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];

        for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
                if (munmap(dev->videoBuffers[i].start,
                          dev->videoBuffers[i].length) == -1) {
                    LOG(WARNING) << "munmap error "
                                 << "errno: " << errno << " error: " << strerror(errno);
                }
            }
            free(dev->videoBuffers);

            if (close(dev->fd) == -1) {
                LOG(WARNING) << "close m_implData->fd error "
                             << "errno: " << errno << " error: " << strerror(errno);
            }

            if (close(dev->sfd) == -1) {
                LOG(WARNING) << "close m_implData->sfd error "
                             << "errno: " << errno << " error: " << strerror(errno);
           }
    }

    eeprom_close(&m_implData->edev);
}

aditof::Status LocalDevice::open() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    struct stat st;
    struct v4l2_capability cap;
    struct VideoDev *dev;

    const char *devName, *subDevName, *cardName;

    std::vector<std::string> paths;
    std::stringstream ss_dev(m_devData.driverPath);
    std::string token;
    while (std::getline(ss_dev, token, '|')) {
        std::stringstream dev_sub(token);
        while (std::getline(dev_sub, token, ';')) {
                paths.push_back(token);
            }
    }

    std::vector<std::string> cards;
    std::stringstream ss_card(CAPTURE_DEVICE_NAME);
    while (std::getline(ss_card, token, '|')) {
        LOG(INFO) << token;
        cards.push_back(token);
    }

    m_implData->videoDevs = new VideoDev[NUM_VIDEO_DEVS];

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
            devName = paths.at(i * 2).c_str();
            subDevName = paths.at(i * 2 + 1).c_str();
        cardName = cards.at(i).c_str();
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

            if (strcmp((char *)cap.card, cardName)) {
                LOG(WARNING) << "CAPTURE Device " << cap.card;
                LOG(WARNING) << "Read " << cardName;
                return Status::GENERIC_ERROR;
            }

            if (!(cap.capabilities &
                  (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
                LOG(WARNING) << devName << " is not a video capture device";
                return Status::GENERIC_ERROR;
            }

            if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
                dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            } else {
                dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
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

            dev->sfd = ::open(subDevName, O_RDWR | O_NONBLOCK, 0);
            if (dev->sfd == -1) {
                LOG(WARNING) << "Cannot open " << subDevName << " errno: " << errno
                             << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }
    }

    if (eeprom_open(EEPROM_DEV_PATH, &m_implData->edev) < 0) {
        LOG(WARNING) << "EEPROM not available!";
    }

    return status;
}

aditof::Status LocalDevice::start() {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;
    struct v4l2_buffer buf;

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];
            if (dev->started) {
                LOG(INFO) << "Device already started";
                return Status::BUSY;
            }
            LOG(INFO) << "Starting device " << i;

            for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
                CLEAR(buf);
                buf.type = dev->videoBuffersType;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;
                buf.m.planes = dev->planes;
                buf.length = 1;

                if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
                    LOG(WARNING) << "mmap error "
                                 << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }
            }

        if (xioctl(dev->fd, VIDIOC_STREAMON,
               &dev->videoBuffersType) != 0) {
                LOG(WARNING) << "VIDIOC_STREAMON error "
                             << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

        dev->started = true;
    }

    return status;
}

aditof::Status LocalDevice::stop() {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];

            if (!dev->started) {
                LOG(INFO) << "Device " << i << " already stopped";
                return Status::BUSY;
            }
            LOG(INFO) << "Stopping device";

            if (xioctl(dev->fd, VIDIOC_STREAMOFF,
                       &dev->videoBuffersType) != 0) {
                LOG(WARNING) << "VIDIOC_STREAMOFF error "
                             << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            dev->started = false;
    }

    return status;
}

aditof::Status
LocalDevice::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    FrameDetails details;

    details.width = 640;
    details.height = 480;
    details.type = "depth_ir";
    types.push_back(details);

    details.width = 640;
    details.height = 480;
    details.type = "depth_only";
    types.push_back(details);

    details.width = 640;
    details.height = 480;
    details.type = "ir_only";
    types.push_back(details);

    details.width = 668;
    details.height = 750;
    details.type = "raw";
    types.push_back(details);

    return status;
}

aditof::Status LocalDevice::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    struct v4l2_requestbuffers req;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    size_t length, offset;

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];
            if (details != m_implData->frameDetails) {
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

            /* Set the frame format in the driver */
            CLEAR(fmt);
            fmt.type = dev->videoBuffersType;
            fmt.fmt.pix.width = details.width;
            fmt.fmt.pix.height = details.height;

            if (xioctl(dev->fd, VIDIOC_S_FMT, &fmt) == -1) {
                LOG(WARNING) << "Setting Pixel Format error, errno: " << errno
                             << " error: " << strerror(errno);
                       return Status::GENERIC_ERROR;
            }

            /* Allocate the video buffers in the driver */
            CLEAR(req);
            req.count = 4;
            req.type = dev->videoBuffersType;
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
                buf.type = dev->videoBuffersType;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = dev->nVideoBuffers;
                buf.m.planes = dev->planes;
                buf.length = 1;

                if (xioctl(dev->fd, VIDIOC_QUERYBUF, &buf) == -1) {
                            LOG(WARNING) << "VIDIOC_QUERYBUF error "
                                 << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }

                if (dev->videoBuffersType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                            length = buf.length;
                            offset = buf.m.offset;
                } else {
                            length = buf.m.planes[0].length;
                            offset = buf.m.planes[0].m.mem_offset;
                }

                dev->videoBuffers[dev->nVideoBuffers].start =
                            mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED,
                         dev->fd, offset);

                if (dev->videoBuffers[dev->nVideoBuffers].start ==
                            MAP_FAILED) {
                            LOG(WARNING) << "mmap error "
                                 << "errno: " << errno << " error: " << strerror(errno);
                            return Status::GENERIC_ERROR;
                }

                dev->videoBuffers[dev->nVideoBuffers].length = length;
            }
    }

    m_implData->frameDetails = details;

    return status;
}

aditof::Status LocalDevice::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static unsigned char buf[CTRL_PACKET_SIZE];
    size_t readBytes = 0;

    if (size <= CTRL_PACKET_SIZE) {
        memset(buf + size, 0, CTRL_PACKET_SIZE);
        memcpy(buf, firmware, size);
        extCtrl.size = 2048 * sizeof(unsigned short);
        extCtrl.p_u16 = (unsigned short *)buf;
        extCtrl.id = V4L2_CID_AD_DEV_SET_CHIP_CONFIG;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        extCtrls.controls = &extCtrl;
        extCtrls.count = 1;

        if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    } else {
        while (readBytes < size) {

            if ((size - readBytes) >= CTRL_PACKET_SIZE) {
                extCtrl.size = 2048 * sizeof(unsigned short);
                extCtrl.p_u16 =
                    (unsigned short *)((char *)firmware + readBytes);
                extCtrl.id = V4L2_CID_AD_DEV_SET_CHIP_CONFIG;
                memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
                extCtrls.controls = &extCtrl;
                extCtrls.count = 1;

                if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) ==
                    -1) {
                    LOG(WARNING)
                        << "Programming AFE error "
                        << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }
                readBytes += CTRL_PACKET_SIZE;
                usleep(100);
            } else {
                memset(buf, 0, CTRL_PACKET_SIZE);
                memcpy(buf, ((const char *)firmware + readBytes),
                       size - readBytes);
                extCtrl.size = 2048 * sizeof(unsigned short);
                extCtrl.p_u16 = (unsigned short *)buf;
                extCtrl.id = V4L2_CID_AD_DEV_SET_CHIP_CONFIG;
                memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
                extCtrls.controls = &extCtrl;
                extCtrls.count = 1;

                if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) ==
                    -1) {
                    LOG(WARNING)
                        << "Programming AFE error "
                        << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }
                readBytes += CTRL_PACKET_SIZE;
                usleep(100);
            }
        }
    }

    return status;
}

aditof::Status LocalDevice::getFrame(uint16_t *buffer) {
    using namespace aditof;
    struct v4l2_buffer buf[NUM_VIDEO_DEVS];
    struct VideoDev *dev;
    Status status;

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];
        status = waitForBuffer(dev);
        if (status != Status::OK) {
                return status;
        }

        status = dequeueInternalBuffer(buf[i], dev);
        if (status != Status::OK) {
                return status;
        }
    }

    unsigned int width;
    unsigned int height;
    unsigned int buf_data_len;
    uint8_t *pdata[NUM_VIDEO_DEVS];

    width = m_implData->frameDetails.width;
    height = m_implData->frameDetails.height;

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];
        status = getInternalBuffer(&pdata[i], buf_data_len, buf[i], dev);
        if (status != Status::OK) {
                return status;
        }
    }

    auto isBufferPacked = [](const struct v4l2_buffer &buf, unsigned int width,
                             unsigned int height) {
        unsigned int bytesused = 0;
        if (buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            bytesused = buf.m.planes[0].bytesused;
        } else {
            bytesused = buf.bytesused;
        }

        return bytesused == (width * height * 3 / 2);
    };

    if ((width == 668)) {
        unsigned int j = 0;
        for (unsigned int i = 0; i < (buf_data_len); i += 3) {
            if ((i != 0) && (i % (336 * 3) == 0)) {
                j -= 4;
            }

            buffer[j] = (((unsigned short)*(pdata[0] + i)) << 4) |
                        (((unsigned short)*(pdata[0] + i + 2)) & 0x000F);
            j++;

            buffer[j] = (((unsigned short)*(pdata[0] + i + 1)) << 4) |
                        ((((unsigned short)*(pdata[0] + i + 2)) & 0x00F0) >> 4);
            j++;
        }
    } else if (!isBufferPacked(buf[0], width, height)) {
        // TODO: investigate optimizations for this (arm neon / 1024 bytes
        // chunks)
        if (m_implData->frameDetails.type == "depth_only") {
            memcpy(buffer, pdata[0], buf[0].bytesused);
        } else if (m_implData->frameDetails.type == "ir_only") {
            memcpy(buffer + (width * height) / 2, pdata[0], buf[0].bytesused);
        // Not Packed and type == "depth_ir"
        } else {
            uint16_t* ptr_depth = (uint16_t*)pdata[0];
            uint16_t* ptr_ir = (uint16_t*)pdata[1];
            uint16_t* ptr_buff_depth = buffer;
            uint16_t* ptr_buff_ir = buffer + (width * height);
            //discard 4 LSB of depth (due to Nvidia RAW memory storage type)
            for (unsigned int k = 0; k < buf[0].bytesused / 2; k+=2) {
		ptr_buff_depth[k] = (*(ptr_depth + k) >> 4);
		ptr_buff_depth[k + 1] = (*(ptr_depth + k + 1) >> 4);
	    }
	    for (unsigned int k = 0; k < buf[0].bytesused / 2; k+=2) {
		ptr_buff_ir[k] = (*(ptr_ir + k) >> 4);
		ptr_buff_ir[k + 1] = (*(ptr_ir + k + 1) >> 4);
	    }
        }
    } else {
        // clang-format off
        uint16_t *depthPtr = buffer;
        uint16_t *irPtr = buffer + (width * height) / 2;
        unsigned int j = 0;

        if (m_implData->frameDetails.type == "depth_only" ||
                m_implData->frameDetails.type == "ir_only") {
                buf_data_len /= 2;
        }
        /* The frame is read from the device as an array of uint8_t's where
         * every 3 uint8_t's can produce 2 uint16_t's that have only 12 bits
         * in use.
         * Ex: consider uint8_t a, b, c;
         * We first convert a, b, c to uint16_t
         * We obtain uint16_t f1 = (a << 4) | (c & 0x000F)
         * and uint16_t f2 = (b << 4) | ((c & 0x00F0) >> 4);
         */
        for (unsigned int i = 0; i < (buf_data_len); i += 24) {
            /* Read 24 bytes from pdata and deinterleave them in 3 separate 8 bytes packs
             *                                   |-> a1 a2 a3 ... a8
             * a1 b1 c1 a2 b2 c2 ... a8 b8 c8  ->|-> b1 b2 b3 ... b8
             *                                   |-> c1 c2 c3 ... c8
             * then convert all the values to uint16_t
             */
            uint8x8x3_t data = vld3_u8(pdata[0]);
            uint16x8_t aData = vmovl_u8(data.val[0]);
            uint16x8_t bData = vmovl_u8(data.val[1]);
            uint16x8_t cData = vmovl_u8(data.val[2]);

            uint16x8_t lowMask = vdupq_n_u16(0x000F);
            uint16x8_t highMask = vdupq_n_u16(0x00F0);

            /* aBuffer = (a << 4) | (c & 0x000F) for every a and c value*/
            uint16x8_t aBuffer = vorrq_u16(vshlq_n_u16(aData, 4), vandq_u16(cData, lowMask));

            /* bBuffer = (b << 4) | ((c & 0x00F0) >> 4) for every b and c value*/
            uint16x8_t bBuffer = vorrq_u16(vshlq_n_u16(bData, 4), vshrq_n_u16(vandq_u16(cData, highMask), 4));

            uint16x8x2_t toStore;
            toStore.val[0] = aBuffer;
            toStore.val[1] = bBuffer;

            if (m_implData->frameDetails.type == "depth_only") {
                vst2q_u16(depthPtr, toStore);
                depthPtr += 16;
            } else if (m_implData->frameDetails.type == "ir_only") {
                vst2q_u16(irPtr, toStore);
                irPtr += 16;
            } else {
                /* Store the 16 frame pixel in the corresponding image */
                if ((j / width) % 2) {
                    vst2q_u16(irPtr, toStore);
                    irPtr += 16;
                } else {
                    vst2q_u16(depthPtr, toStore);
                    depthPtr += 16;
                }
            }
            j += 16;
            pdata[0] += 24;
        }
        // clang-format on
    }

    for (unsigned int i = 0; i < NUM_VIDEO_DEVS; i++) {
        dev = &m_implData->videoDevs[i];
            status = enqueueInternalBuffer(buf[i], dev);
            if (status != Status::OK) {
                return status;
    }
    }

    return status;
}

aditof::Status LocalDevice::readEeprom(uint32_t address, uint8_t *data,
                                       size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    if (!m_implData->edev.valid) {
        LOG(WARNING) << "EEPROM not available!";
        return Status::GENERIC_ERROR;
    }

    int ret = eeprom_read_buf(&m_implData->edev, address, data, length);
    if (ret == -1) {
        LOG(WARNING) << "EEPROM read error";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status LocalDevice::writeEeprom(uint32_t address, const uint8_t *data,
                                        size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    if (!m_implData->edev.valid) {
        LOG(WARNING) << "EEPROM not available!";
        return Status::GENERIC_ERROR;
    }

    int ret = eeprom_write_buf(&m_implData->edev, address,
                               const_cast<uint8_t *>(data), length);
    if (ret == -1) {
        LOG(WARNING) << "EEPROM write error";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status LocalDevice::readAfeRegisters(const uint16_t *address,
                                             uint16_t *data, size_t length) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;

    extCtrl.size = 2048 * sizeof(unsigned short);

    for (size_t i = 0; i < length; i++) {
        extCtrl.p_u16 = const_cast<uint16_t *>(&address[i]);
        extCtrl.id = V4L2_CID_AD_DEV_READ_REG;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        extCtrls.controls = &extCtrl;
        extCtrls.count = 1;

        if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
        data[i] = *extCtrl.p_u16;
    }

    return status;
}

aditof::Status LocalDevice::writeAfeRegisters(const uint16_t *address,
                                              const uint16_t *data,
                                              size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    static unsigned char buf[CTRL_PACKET_SIZE];
    unsigned short sampleCnt = 0;

    length *= 2 * sizeof(unsigned short);
    while (length) {
        memset(buf, 0, CTRL_PACKET_SIZE);
        size_t maxBytesToSend =
            length > CTRL_PACKET_SIZE ? CTRL_PACKET_SIZE : length;
        for (size_t i = 0; i < maxBytesToSend; i += 4) {
            *(unsigned short *)(buf + i) = address[sampleCnt];
            *(unsigned short *)(buf + i + 2) = data[sampleCnt];
            sampleCnt++;
        }
        length -= maxBytesToSend;

        extCtrl.size = 2048 * sizeof(unsigned short);
        extCtrl.p_u16 = (unsigned short *)buf;
        extCtrl.id = V4L2_CID_AD_DEV_SET_CHIP_CONFIG;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        extCtrls.controls = &extCtrl;
        extCtrls.count = 1;

        if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}

aditof::Status LocalDevice::readAfeTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_deviceDetails.sensorType == aditof::SensorType::SENSOR_96TOF1) {
        temp_sensor tdev;

        if (temp_sensor_open(TEMP_SENSOR_DEV_PATH, AFE_TEMP_SENSOR_I2C_ADDR,
                             &tdev) < 0) {
            LOG(WARNING) << "Temp sensor open error";
            return Status::GENERIC_ERROR;
        }

        if (temp_sensor_read(&tdev, &temperature) == -1) {
            LOG(WARNING) << "Error reading AFE_TEMP_SENSOR";
            return Status::GENERIC_ERROR;
        }

        temp_sensor_close(&tdev);
    } else if ((m_deviceDetails.sensorType ==
                aditof::SensorType::SENSOR_CHICONY) ||
			   (m_deviceDetails.sensorType ==
                aditof::SensorType::SENSOR_FX1)) {
        int fd = ::open(TEMP_SENSOR_DEV_PATH, O_RDONLY);
        if (fd <= 0) {
            LOG(WARNING) << "Temp sensor open error";
            return Status::GENERIC_ERROR;
        }
        char buf[6];
        pread(fd, buf, 6, 0);
        close(fd);
        temperature = atof(buf) / 1000.0f;
    }

    return status;
}

aditof::Status LocalDevice::readLaserTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_deviceDetails.sensorType == aditof::SensorType::SENSOR_96TOF1) {
        temp_sensor tdev;

        if (temp_sensor_open(TEMP_SENSOR_DEV_PATH, LASER_TEMP_SENSOR_I2C_ADDR,
                             &tdev) < 0) {
            LOG(WARNING) << "Temp sensor open error";
            return Status::GENERIC_ERROR;
        }

        if (temp_sensor_read(&tdev, &temperature) == -1) {
            LOG(WARNING) << "Error reading LASER_TEMP_SENSOR";
            return Status::GENERIC_ERROR;
        }

        temp_sensor_close(&tdev);
    } else if ((m_deviceDetails.sensorType ==
                aditof::SensorType::SENSOR_CHICONY) ||
               (m_deviceDetails.sensorType ==
                aditof::SensorType::SENSOR_FX1)) {
        int fd = ::open(TEMP_SENSOR_DEV_PATH, O_RDONLY);
        if (fd <= 0) {
            LOG(WARNING) << "Temp sensor open error";
            return Status::GENERIC_ERROR;
        }
        char buf[6];
        pread(fd, buf, 6, 0);
        close(fd);
        temperature = atof(buf) / 1000.0f;
    }

    return status;
}

aditof::Status LocalDevice::getDetails(aditof::DeviceDetails &details) const {
    details = m_deviceDetails;
    return aditof::Status::OK;
}

aditof::Status LocalDevice::waitForBuffer(struct VideoDev *dev = nullptr) {
    fd_set fds;
    struct timeval tv;
    int r;

    if (dev == nullptr)
       struct VideoDev *dev = &m_implData->videoDevs[0];

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

aditof::Status LocalDevice::dequeueInternalBuffer(struct v4l2_buffer &buf,
                                                  struct VideoDev *dev = nullptr) {
    using namespace aditof;
    Status status = Status::OK;

    if (dev == nullptr)
       struct VideoDev *dev = &m_implData->videoDevs[0];

    CLEAR(buf);
    buf.type = dev->videoBuffersType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = dev->planes;

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

aditof::Status LocalDevice::getInternalBuffer(uint8_t **buffer,
                                              uint32_t &buf_data_len,
                                              const struct v4l2_buffer &buf,
                                              struct VideoDev *dev = nullptr) {
     if (dev == nullptr)
        struct VideoDev *dev = &m_implData->videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = m_implData->frameDetails.width *
                   m_implData->frameDetails.height * 3 / 2;

    return aditof::Status::OK;
}

aditof::Status LocalDevice::enqueueInternalBuffer(struct v4l2_buffer &buf,
                                        struct VideoDev *dev = nullptr ) {
    if (dev == nullptr)
        struct VideoDev *dev = &m_implData->videoDevs[0];

    if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

aditof::Status LocalDevice::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    if (dev->fd != -1) {
        fileDescriptor = dev->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}
