#include <aditof/camera_96tof1_specifics.h>
#include <glog/logging.h>

#include "camera_96tof1.h"

using namespace aditof;

Camera96Tof1Specifics::Camera96Tof1Specifics(Camera *camera)
    : m_camera(dynamic_cast<Camera96Tof1 *>(camera)), m_noiseReductionOn(false),
      m_noiseReductionThreshold(0) {

    if (!m_camera) {
        LOG(ERROR) << "Cannot cast camera to a Camera96Tof1";
    }
}

Status Camera96Tof1Specifics::enableNoiseReduction(bool en) {
    aditof::Status status = setTresholdAndEnable(m_noiseReductionThreshold, en);

    if (status == Status::OK) {
        m_noiseReductionOn = en;
    }

    return status;
}

bool Camera96Tof1Specifics::noiseReductionEnabled() const {
    return m_noiseReductionOn;
}

Status Camera96Tof1Specifics::setNoiseReductionThreshold(uint16_t threshold) {

    aditof::Status status = setTresholdAndEnable(threshold, m_noiseReductionOn);

    if (status == Status::OK) {
        m_noiseReductionThreshold = threshold;
    }

    return status;
}

uint16_t Camera96Tof1Specifics::noiseReductionThreshold() const {
    return m_noiseReductionThreshold;
}

Status Camera96Tof1Specifics::setTresholdAndEnable(uint16_t treshold, bool en) {
    const size_t REGS_CNT = 5;
    uint16_t afeRegsAddr[REGS_CNT] = {0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22};
    uint16_t afeRegsVal[REGS_CNT] = {0x0006, 0x0004, 0, 0x0007, 0x0004};

    afeRegsVal[2] |= treshold;
    if (en) {
        afeRegsVal[2] |= 0x8000;
    }

    return m_camera->m_device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
}
