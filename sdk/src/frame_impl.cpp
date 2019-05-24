#include "frame_impl.h"
#include "frame_operations.h"

#include <cmath>
#include <fstream>
#include <glog/logging.h>

FrameImpl::FrameImpl()
    : m_details{0, 0, "", {0.0f, 1.0f}}, m_depthData(nullptr),
      m_irData(nullptr), m_rawData(nullptr) {}

FrameImpl::~FrameImpl() {
    if (m_rawData) {
        delete[] m_rawData;
        m_rawData = nullptr;
    }
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
    m_rawData = new uint16_t[details.width * details.height];
    m_details = details;

    m_depthData = m_rawData;
    m_irData = m_rawData + (details.width * details.height) / 2;

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
        float offset = m_details.cal_data.offset;
        float gain = m_details.cal_data.gain;

        auto equal = [](float a, float b) -> bool {
            return fabs(a - b) < 1e-6;
        };

        if (!equal(gain, 1.0f) || !equal(offset, 0.0f)) {
            for (unsigned int i = 0; i < m_details.width * m_details.height / 2;
                 i++) {
                m_depthData[i] = static_cast<uint16_t>(
                    static_cast<float>(m_depthData[i]) * gain + offset);
            }
        }
        *dataPtr = m_depthData;
        break;
    }
    }

    return Status::OK;
}
