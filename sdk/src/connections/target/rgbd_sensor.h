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
#ifndef RGBD_SENSOR_H
#define RGBD_SENSOR_H

#include "addi9036_sensor.h"
#include "aditof/depth_sensor_interface.h"
#include "rgb_sensor.h"
#include <mutex>

#include <memory>
class RgbdSensor : public aditof::DepthSensorInterface {
  public:
    RgbdSensor();
    RgbdSensor(std::shared_ptr<DepthSensorInterface> depthSensor,
               std::shared_ptr<DepthSensorInterface> rgbSensor);
    ~RgbdSensor();

  public: // implements DepthSensorInterface
    virtual aditof::Status open() override;
    virtual aditof::Status start() override;
    virtual aditof::Status stop() override;
    virtual aditof::Status
    getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) override;
    virtual aditof::Status
    setFrameType(const aditof::FrameDetails &details) override;
    virtual aditof::Status program(const uint8_t *firmware,
                                   size_t size) override;
    virtual aditof::Status getFrame(uint16_t *buffer,
                                    aditof::BufferInfo *bufferInfo) override;
    virtual aditof::Status readAfeRegisters(const uint16_t *address,
                                            uint16_t *data,
                                            size_t length) override;
    virtual aditof::Status writeAfeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length) override;
    virtual aditof::Status
    getDetails(aditof::SensorDetails &details) const override;
    virtual aditof::Status getHandle(void **handle) override;
    virtual aditof::Status getName(std::string &sensorName) const override;

  private:
    std::string m_sensorName;
    int m_id;
    aditof::FrameDetails m_details;
    aditof::SensorDetails m_sensorDetails;
    std::shared_ptr<DepthSensorInterface> m_depthSensor;
    std::shared_ptr<DepthSensorInterface> m_rgbSensor;
    std::mutex m_mtxDepth;
    std::mutex m_mtxRgb;

    uint16_t *m_bufferDepth;
    uint16_t *m_bufferRgb;
    aditof::BufferInfo m_bufferInfo;
};

#endif // RGBD_SENSOR_SENSOR_H
