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
#include "rgbd_sensor.h"
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
#include <linux/videodev2.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unordered_map>

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

RgbdSensor::RgbdSensor(std::shared_ptr<DepthSensorInterface> depthSensor,
                       std::shared_ptr<DepthSensorInterface> rgbSensor)
    : m_depthSensor(depthSensor), m_rgbSensor(rgbSensor) {
    m_sensorName = "rgbd";
}

RgbdSensor::~RgbdSensor() = default;

aditof::Status RgbdSensor::open() {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->open();
    status = m_rgbSensor->open();
    return status;
}

aditof::Status RgbdSensor::start() {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->start();
    status = m_rgbSensor->start();
    return status;
}

aditof::Status RgbdSensor::stop() {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->stop();
    status = m_rgbSensor->stop();
    return status;
}

aditof::Status
RgbdSensor::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    FrameDetails details;
    memset(&details, 0, sizeof(FrameDetails));

    details.width = aditof::FRAME_WIDTH;
    details.height = aditof::FRAME_HEIGHT;
    details.rgbWidth = aditof::RGB_FRAME_WIDTH;
    details.rgbHeight = aditof::RGB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height * 2;
    details.type = "depth_ir_rgb";
    types.push_back(details);

    details.width = aditof::FRAME_WIDTH;
    details.height = aditof::FRAME_HEIGHT;
    details.rgbWidth = aditof::RGB_FRAME_WIDTH;
    details.rgbHeight = aditof::RGB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height;
    details.type = "depth_rgb";
    types.push_back(details);

    details.width = aditof::FRAME_WIDTH;
    details.height = aditof::FRAME_HEIGHT;
    details.rgbWidth = aditof::RGB_FRAME_WIDTH;
    details.rgbHeight = aditof::RGB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height;
    details.type = "ir_rgb";
    types.push_back(details);

    return status;
}

aditof::Status RgbdSensor::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;
    aditof::FrameDetails tofDetails = details;
    aditof::FrameDetails rgbDetails = details;
    m_details = details;

    tofDetails.type = details.type.substr(0, details.type.find("_rgb"));
    status = m_depthSensor->setFrameType(tofDetails);

    rgbDetails.type = "rgb";
    status = m_rgbSensor->setFrameType(tofDetails);

    return status;
}

aditof::Status RgbdSensor::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->program(firmware, size);
    return status;
}

aditof::Status RgbdSensor::getFrame(uint16_t *buffer,
                                    aditof::BufferInfo *bufferInfo) {

    using namespace aditof;
    uint16_t *depthDataLocation = buffer;
    uint16_t *rgbDataLocation =
        (buffer + m_details.fullDataHeight * m_details.fullDataWidth);

    aditof::BufferInfo depthInfo;
    aditof::BufferInfo rgbInfo;

    std::vector<std::thread> threads;
    threads.push_back(std::thread(&DepthSensorInterface::getFrame,
                                  m_rgbSensor.get(), rgbDataLocation,
                                  &rgbInfo));

    threads.push_back(std::thread(&DepthSensorInterface::getFrame,
                                  m_depthSensor.get(), depthDataLocation,
                                  &depthInfo));
    threads[0].join();
    threads[1].join();

    return aditof::Status::OK;
}

aditof::Status RgbdSensor::readAfeRegisters(const uint16_t *address,
                                            uint16_t *data, size_t length) {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->readAfeRegisters(address, data, length);
    return status;
}

aditof::Status RgbdSensor::writeAfeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length) {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->writeAfeRegisters(address, data, length);
    return status;
}

aditof::Status RgbdSensor::getDetails(aditof::SensorDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->getDetails(details);
    status = m_rgbSensor->getDetails(details);
    return status;
}

aditof::Status RgbdSensor::getHandle(void **handle) {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->getHandle(handle);
    return status;
}

aditof::Status RgbdSensor::getName(std::string &sensorName) const {

    sensorName = m_sensorName;
    return aditof::Status::OK;
}
