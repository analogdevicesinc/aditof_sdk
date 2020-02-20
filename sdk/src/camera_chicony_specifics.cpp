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
