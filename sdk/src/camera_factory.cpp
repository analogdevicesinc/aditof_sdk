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
#include "camera_factory.h"
#include "camera_96tof1.h"
#ifdef CHICONY_006
#include "camera_chicony_006.h"
#endif
#ifdef FX1
#include "camera_fx1.h"
#endif
#include <aditof/device_definitions.h>

namespace aditof {

std::unique_ptr<Camera>
CameraFactory::buildCamera(std::unique_ptr<DeviceInterface> device) {
    using namespace aditof;

    DeviceDetails devDetails;
    device->getDetails(devDetails);
    switch (devDetails.sensorType) {

    case SensorType::SENSOR_96TOF1:
        return std::unique_ptr<Camera>(new Camera96Tof1(std::move(device)));

    case SensorType::SENSOR_CHICONY:
#ifdef CHICONY_006
        return std::unique_ptr<Camera>(new CameraChicony(std::move(device)));
#else
        return nullptr;
#endif
    case SensorType::SENSOR_FX1:
#ifdef FX1
        return std::unique_ptr<Camera>(new CameraFx1(std::move(device)));
#else
        return nullptr;
#endif
    }

    return nullptr;
}

} // namespace aditof
