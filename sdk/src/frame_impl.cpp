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
#include "frame_impl.h"
#include <aditof/frame_operations.h>

#include <cmath>
#include <cstring>
#include <fstream>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

FrameImpl::FrameImpl()
    : m_details{0, 0, 0, 0, 0, 0, ""}, m_depthData(nullptr), m_irData(nullptr),
      m_rgbData(nullptr), m_fullData(nullptr) {}

FrameImpl::~FrameImpl() {
    if (m_fullData) {
        delete[] m_fullData;
        m_fullData = nullptr;
    }
}

FrameImpl::FrameImpl(const FrameImpl &op) {
    allocFrameData(op.m_details);
    memcpy(m_fullData, op.m_fullData,
           sizeof(uint16_t) * op.m_details.fullDataWidth *
               op.m_details.fullDataHeight);

    m_details = op.m_details;
}

FrameImpl &FrameImpl::operator=(const FrameImpl &op) {
    if (this != &op) {
        if (m_fullData) {
            delete[] m_fullData;
            m_fullData = nullptr;
        }
        allocFrameData(op.m_details);
        memcpy(m_fullData, op.m_fullData,
               sizeof(uint16_t) * op.m_details.fullDataWidth *
                   op.m_details.fullDataHeight);
        m_details = op.m_details;
    }

    return *this;
}

aditof::Status FrameImpl::setDetails(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    if (details == m_details) {
        LOG(INFO) << "Same details provided. Doing nothing.";
        return status;
    }

    if (m_fullData) {
        delete[] m_fullData;
        m_fullData = nullptr;
    }

    m_details = details;
    allocFrameData(m_details);

    return status;
}

aditof::Status FrameImpl::getDetails(aditof::FrameDetails &details) const {
    details = m_details;

    return aditof::Status::OK;
}

aditof::Status FrameImpl::getData(aditof::FrameDataType dataType,
                                  uint16_t **dataPtr) {
    using namespace aditof;

    if (dataPtr == nullptr) {
        LOG(ERROR) << "Received dataPtr null pointer";
        return aditof::Status::INVALID_ARGUMENT;
    }

    switch (dataType) {
    case FrameDataType::FULL_DATA: {
        *dataPtr = m_fullData;
        break;
    }
    case FrameDataType::IR: {
        *dataPtr = m_irData;
        break;
    }
    case FrameDataType::DEPTH: {
        *dataPtr = m_depthData;
        break;
    }
    case FrameDataType::RGB: {
        *dataPtr = m_rgbData;
        break;
    }
    }

    return Status::OK;
}

void FrameImpl::allocFrameData(const aditof::FrameDetails &details) {

    m_fullData = new uint16_t[details.fullDataWidth * details.fullDataHeight +
                              details.rgbWidth * details.rgbHeight];
    m_depthData = m_fullData;
    if (details.fullDataHeight == details.height)
        m_irData = m_depthData;
    else
        m_irData = m_fullData + (details.width * details.height);

    m_rgbData = m_fullData + (details.fullDataWidth * details.fullDataHeight);
}
