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
#include "local_device_addi903x.h"
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

#define V4L2_CID_AD_DEV_SET_CHIP_CONFIG 0xA00A00
#define V4L2_CID_AD_DEV_READ_REG 0xA00A01
#define CTRL_PACKET_SIZE 4096

#define LASER_TEMP_SENSOR_I2C_ADDR 0x49
#define AFE_TEMP_SENSOR_I2C_ADDR 0x4b

// TO DO: This exists in linux_utils.h which is not included on Dragoboard.
// Should not have duplicated code if possible.
static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

LocalDeviceADDI903x::LocalDeviceADDI903x(const aditof::DeviceConstructionData &data)
    : LocalDevice(data) {
}

LocalDeviceADDI903x::~LocalDeviceADDI903x() {

}

aditof::Status
LocalDeviceADDI903x::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    FrameDetails details;

    details.width = 640;
    details.height = 960;
    details.cal_data.offset = 0;
    details.cal_data.gain = 1;
    details.type = "depth_ir";
    types.push_back(details);

    details.width = 640;
    details.height = 960;
    details.cal_data.offset = 0;
    details.cal_data.gain = 1;
    details.type = "depth_only";
    types.push_back(details);

    details.width = 640;
    details.height = 960;
    details.cal_data.offset = 0;
    details.cal_data.gain = 1;
    details.type = "ir_only";
    types.push_back(details);

    return status;
}

aditof::Status LocalDeviceADDI903x::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;

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

        if (xioctl(m_implData->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
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

                if (xioctl(m_implData->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) ==
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

                if (xioctl(m_implData->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) ==
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

aditof::Status LocalDeviceADDI903x::getFrame(uint16_t *buffer) {
    using namespace aditof;

    Status status = waitForBuffer();
    if (status != Status::OK) {
        return status;
    }

    struct v4l2_buffer buf;

    status = dequeueInternalBuffer(buf);
    if (status != Status::OK) {
        return status;
    }

    unsigned int width;
    unsigned int height;
    unsigned int buf_data_len;
    uint8_t *pdata;

    width = m_implData->frameDetails.width;
    height = m_implData->frameDetails.height;

    status = getInternalBuffer(&pdata, buf_data_len, buf);
    if (status != Status::OK) {
        return status;
    }

	if (buf.bytesused != (width * height * 3 / 2)) {
        // TODO: investigate optimizations for this (arm neon / 1024 bytes
        // chunks)
        if (m_implData->frameDetails.type == "depth_only") {
            memcpy(buffer, pdata, buf.bytesused);
        } else if (m_implData->frameDetails.type == "ir_only") {
            memcpy(buffer + (width * height) / 2, pdata, buf.bytesused);
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
            uint8x8x3_t data = vld3_u8(pdata);
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
            pdata += 24;
        }
        // clang-format on
    }

    status = enqueueInternalBuffer(buf);
    if (status != Status::OK) {
        return status;
    }

    return status;
}

aditof::Status LocalDevice::readAfeRegisters(const uint16_t *address,
                                             uint16_t *data, size_t length) {
    using namespace aditof;
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

        if (xioctl(m_implData->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
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

        if (xioctl(m_implData->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}