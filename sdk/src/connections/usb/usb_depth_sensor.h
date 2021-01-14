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
#ifndef USB_DEPTH_SENSOR_H
#define USB_DEPTH_SENSOR_H

#include "aditof/depth_sensor_interface.h"
#include "aditof/sensor_definitions.h"

#include <memory>
namespace aditof {
//TODO this is temporary and should be removed imediately after these details are read from the hardware
static const unsigned int USB_FRAME_WIDTH = 640;
static const unsigned int USB_FRAME_HEIGHT = 480;
} // namespace aditof

class UsbDepthSensor : public aditof::DepthSensorInterface {
  public:
    UsbDepthSensor(const std::string &driverPath);
    ~UsbDepthSensor();

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
    virtual aditof::Status getFrame(uint16_t *buffer) override;
    virtual aditof::Status readAfeRegisters(const uint16_t *address,
                                            uint16_t *data,
                                            size_t length) override;
    virtual aditof::Status writeAfeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length) override;
    virtual aditof::Status
    getDetails(aditof::SensorDetails &details) const override;
    virtual aditof::Status getHandle(void **handle) override;

  private:
    struct ImplData;

    aditof::SensorDetails m_sensorDetails;
    std::string m_driverPath;
    std::unique_ptr<ImplData> m_implData;
};

#endif // USB_DEPTH_SENSOR_H
