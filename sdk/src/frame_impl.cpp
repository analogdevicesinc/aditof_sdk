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

FrameImpl::FrameImpl() : m_allData(nullptr) {}

FrameImpl::~FrameImpl() {
    if (m_allData) {
        delete[] m_allData;
        m_allData = nullptr;
    }
}

FrameImpl::FrameImpl(const FrameImpl &op) {
    allocFrameData(op.m_details);
    memcpy(m_allData, op.m_allData,
           sizeof(uint16_t) * op.m_details.width * op.m_details.height);
    m_details = op.m_details;
}

FrameImpl &FrameImpl::operator=(const FrameImpl &op) {
    if (this != &op) {
        if (m_allData) {
            delete[] m_allData;
            m_allData = nullptr;
        }
        allocFrameData(op.m_details);
        memcpy(m_allData, op.m_allData,
               sizeof(uint16_t) * op.m_details.width * op.m_details.height);
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

    if (m_allData) {
        delete[] m_allData;
        m_allData = nullptr;
    }
    allocFrameData(details);
    m_details = details;

    return status;
}

aditof::Status FrameImpl::getDetails(aditof::FrameDetails &details) const {
    details = m_details;

    return aditof::Status::OK;
}

aditof::Status FrameImpl::getData(const std::string &dataType,
                                  uint16_t **dataPtr) {
    using namespace aditof;

    if (m_dataLocations.count(dataType) > 0) {
        *dataPtr = m_dataLocations[dataType];
    } else {
        dataPtr = nullptr;
        LOG(ERROR) << dataType << " is not supported by this frame!";
        return Status::INVALID_ARGUMENT;
    }

    return Status::OK;
}

void FrameImpl::allocFrameData(const aditof::FrameDetails &details) {
    unsigned int totalWidth = 0;
    unsigned int totalHeigth = 0;

    for (const auto &details : details.dataDetails) {
        totalWidth += details.width;
        totalHeigth += details.height;
    }

    m_allData = new uint16_t[totalWidth * totalHeigth];
    m_dataLocations.emplace("allData", m_allData);

    unsigned int pos = 0;
    for (const auto &details : details.dataDetails) {
        m_dataLocations.emplace(details.type, m_allData + pos);
        pos += details.width * details.height;
    }
}
