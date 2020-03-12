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
#include <aditof/camera_chicony_specifics.h>
#include <aditof/device_interface.h>
#include <glog/logging.h>

#include "camera_chicony.h"

using namespace aditof;

CameraChiconySpecifics::CameraChiconySpecifics(Camera *camera)
    : m_camera(dynamic_cast<CameraChicony *>(camera)),
      m_noiseReductionOn(false), m_noiseReductionThreshold(0) {
    if (!m_camera) {
        LOG(ERROR) << "Cannot cast camera to a CameraChicony";
    }
}

Status CameraChiconySpecifics::enableNoiseReduction(bool en) {
    aditof::Status status = setTresholdAndEnable(m_noiseReductionThreshold, en);

    if (status == Status::OK) {
        m_noiseReductionOn = en;
    }

    return status;
}

bool CameraChiconySpecifics::noiseReductionEnabled() const {
    return m_noiseReductionOn;
}

Status CameraChiconySpecifics::setNoiseReductionThreshold(uint16_t threshold) {

    aditof::Status status = setTresholdAndEnable(threshold, m_noiseReductionOn);

    if (status == Status::OK) {
        m_noiseReductionThreshold = threshold;
    }

    return status;
}

uint16_t CameraChiconySpecifics::noiseReductionThreshold() const {
    return m_noiseReductionThreshold;
}

Status CameraChiconySpecifics::setTresholdAndEnable(uint16_t treshold,
                                                    bool en) {
    const size_t REGS_CNT = 5;
    uint16_t afeRegsAddr[REGS_CNT] = {0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22};
    uint16_t afeRegsVal[REGS_CNT] = {0x0006, 0x0004, 0, 0x0007, 0x0004};

    afeRegsVal[2] |= treshold;
    if (en) {
        afeRegsVal[2] |= 0x8000;
    }

    return m_camera->m_device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
}
