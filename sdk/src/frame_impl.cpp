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
#include <glog/logging.h>

FrameImpl::FrameImpl()
    : m_details{0, 0, 0, 0, ""}, m_depthData(nullptr), m_irData(nullptr),
      m_rawData(nullptr) {}

FrameImpl::~FrameImpl() {
    if (m_rawData) {
        delete[] m_rawData;
        m_rawData = nullptr;
    }
}

FrameImpl::FrameImpl(const FrameImpl &op) {
    allocFrameData(op.m_details);
    memcpy(m_rawData, op.m_rawData,
           sizeof(uint16_t) * op.m_details.fullDataWidth *
               op.m_details.fullDataHeight);
    m_details = op.m_details;
}

FrameImpl &FrameImpl::operator=(const FrameImpl &op) {
    if (this != &op) {
        if (m_rawData) {
            delete[] m_rawData;
            m_rawData = nullptr;
        }
        allocFrameData(op.m_details);
        memcpy(m_rawData, op.m_rawData,
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

    if (m_rawData) {
        delete[] m_rawData;
        m_rawData = nullptr;
    }
    allocFrameData(details);
    m_details = details;

    return status;
}

aditof::Status FrameImpl::getDetails(aditof::FrameDetails &details) const {
    details = m_details;

    return aditof::Status::OK;
}

aditof::Status FrameImpl::getData(aditof::FrameDataType dataType,
                                  uint16_t **dataPtr) {
    using namespace aditof;

    switch (dataType) {
    case FrameDataType::RAW: {
        *dataPtr = m_rawData;
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
    }

    return Status::OK;
}

void FrameImpl::allocFrameData(const aditof::FrameDetails &details) {
    m_rawData = new uint16_t[details.fullDataWidth * details.fullDataHeight];
    m_depthData = m_rawData;
    m_irData = m_rawData + (details.width * details.height);
}
