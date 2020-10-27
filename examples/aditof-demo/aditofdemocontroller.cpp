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
#include "aditofdemocontroller.h"

#include <glog/logging.h>
#include <iostream>

AdiTofDemoController::AdiTofDemoController()
    : m_cameraInUse(-1), m_frameRequested(false),
      m_recorder(new AditofDemoRecorder()) {
    m_system = new aditof::System();
    m_system->initialize();
    m_system->getCameraList(m_cameras);
    if (m_cameras.size()) {
        // Use the first camera that is found
        m_cameraInUse = 0;
        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
        camera->initialize();
        std::vector<std::string> frameTypes;
        camera->getAvailableFrameTypes(frameTypes);
        if (frameTypes.empty()) {
            LOG(WARNING) << "no frame type available!";
            return;
        }
        camera->setFrameType(frameTypes.front());

        std::vector<std::string> modes;
        camera->getAvailableModes(modes);
        if (modes.empty()) {
            LOG(WARNING) << "no camera modes available!";
            return;
        }

    } else {
        LOG(WARNING) << "No cameras found!";
    }
}

bool AdiTofDemoController::setRegularConnection() {
    delete m_system;
    m_system = new aditof::System();
    m_system->initialize();
    m_system->getCameraList(m_cameras);
    m_IsEthernetConnection = false;
    if (m_cameras.size()) {
        // Use the first camera that is found
        m_cameraInUse = 0;
        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];

        camera->initialize();

        std::vector<std::string> frameTypes;
        camera->getAvailableFrameTypes(frameTypes);
        if (frameTypes.empty()) {
            LOG(WARNING) << "no frame type available!";
            return false;
        }
        camera->setFrameType(frameTypes.front());

        std::vector<std::string> modes;
        camera->getAvailableModes(modes);
        if (modes.empty()) {
            LOG(WARNING) << "no camera modes available!";
            return false;
        }
        return true;

    } else {
        LOG(WARNING) << "No cameras found!";
        return false;
    }
}

bool AdiTofDemoController::setEthernetConnection(const std::string &ip) {
    delete m_system;
    // TO DO: replace the boolean variable m_IsEthernetConnection with a check
    // of the camera type (we need the sdk implementation first)
    if (m_IsEthernetConnection == true)
        m_cameras.clear();
    m_system = new aditof::System();
    m_system->initialize();
    m_system->getCameraListAtIp(m_cameras, ip);
    m_IsEthernetConnection = true;
    if (m_cameras.size()) {
        // Use the first camera that is found
        m_cameraInUse = 0;
        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];

        camera->initialize();

        std::vector<std::string> frameTypes;
        camera->getAvailableFrameTypes(frameTypes);
        if (frameTypes.empty()) {
            LOG(WARNING) << "no frame type available!";
            return false;
        }
        camera->setFrameType(frameTypes.front());

        std::vector<std::string> modes;
        camera->getAvailableModes(modes);
        if (modes.empty()) {
            LOG(WARNING) << "no camera modes available!";
            return false;
        }
        return true;

    } else {
        LOG(WARNING) << "No cameras found!";
        return false;
    }
}

AdiTofDemoController::~AdiTofDemoController() {
    if (m_cameraInUse == -1) {
        return;
    }
    stopCapture();
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->stop();
    delete m_system;
}

void AdiTofDemoController::startCapture() {
    if (m_cameraInUse == -1) {
        return;
    }

    m_stopFlag = false;
    m_workerThread =
        std::thread(std::bind(&AdiTofDemoController::captureFrames, this));
}

void AdiTofDemoController::stopCapture() {
    if (m_cameraInUse == -1) {
        return;
    }
    std::unique_lock<std::mutex> lock(m_requestMutex);
    m_stopFlag = true;
    lock.unlock();
    m_requestCv.notify_one();
    if (m_workerThread.joinable()) {
        m_workerThread.join();
    }
}

std::string AdiTofDemoController::getMode() const {
    // TODO: implement get mode
    return "";
}

void AdiTofDemoController::setMode(const std::string &mode) {
    if (m_cameraInUse == -1) {
        return;
    }
    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
    camera->setMode(mode);
}

std::pair<float, float> AdiTofDemoController::getTemperature() {
    auto returnValue = std::make_pair<float, float>(0.0, 0.0);

    if (m_cameraInUse == -1) {
        return returnValue;
    }

    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];

    std::shared_ptr<aditof::DeviceInterface> device = camera->getDevice();

    device->readAfeTemp(returnValue.first);
    device->readLaserTemp(returnValue.second);

    return returnValue;
}

aditof::Status AdiTofDemoController::writeAFEregister(uint16_t *address,
                                                      uint16_t *data,
                                                      uint16_t noOfEntries) {
    if (m_cameraInUse == -1) {
        return aditof::Status::GENERIC_ERROR;
    }

    auto device =
        m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDevice();
    return device->writeAfeRegisters(address, data, noOfEntries);
}

aditof::Status AdiTofDemoController::readAFEregister(uint16_t *address,
                                                     uint16_t *data,
                                                     uint16_t noOfEntries) {

    auto device =
        m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDevice();
    return device->readAfeRegisters(address, data, noOfEntries);
}

void AdiTofDemoController::startRecording(const std::string &fileName,
                                            const aditof::FrameDetails &frameDetails, unsigned int fps) {
    if (m_cameraInUse == -1) {
        return;
    }
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(cameraDetails);
    m_recorder->startRecording(fileName, frameDetails, cameraDetails, fps);
}

void AdiTofDemoController::stopRecording() { m_recorder->stopRecording(); }

int AdiTofDemoController::startPlayback(const std::string &fileName, int &fps) {
    return m_recorder->startPlayback(fileName, fps);
}

void AdiTofDemoController::stopPlayback() { m_recorder->stopPlayback(); }

bool AdiTofDemoController::playbackFinished() const {
    return m_recorder->isPlaybackFinished();
}

std::shared_ptr<aditof::Frame> AdiTofDemoController::getFrame() {
    if (m_recorder->isPlaybackEnabled()) {
        return m_recorder->readNewFrame();
    }
    return m_queue.dequeue();
}

void AdiTofDemoController::requestFrame() {
    if (m_recorder->isPlaybackEnabled()) {
        m_recorder->requestFrame();
    } else {
        std::unique_lock<std::mutex> lock(m_requestMutex);
        m_frameRequested = true;
        lock.unlock();
        m_requestCv.notify_one();
    }
}

bool AdiTofDemoController::hasCamera() const { return !m_cameras.empty(); }

void AdiTofDemoController::captureFrames() {
    while (!m_stopFlag.load()) {
        std::unique_lock<std::mutex> lock(m_requestMutex);
        m_requestCv.wait(lock, [&] { return m_frameRequested || m_stopFlag; });

        if (m_stopFlag) {
            break;
        }

        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
        auto frame = std::make_shared<aditof::Frame>();
        aditof::Status status = camera->requestFrame(frame.get());
        if (status != aditof::Status::OK) {
            m_frameRequested = false;
            continue;
        }

        if (m_recorder->isRecordingEnabled()) {
            m_recorder->recordNewFrame(frame);
        }

        m_queue.enqueue(frame);
        m_frameRequested = false;
    }
}

int AdiTofDemoController::getRangeMax() const {
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
        cameraDetails);
    return cameraDetails.maxDepth;
}

int AdiTofDemoController::getRangeMin() const {
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
        cameraDetails);
    return cameraDetails.minDepth;
}

int AdiTofDemoController::getbitCount() const {
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
        cameraDetails);
    return cameraDetails.bitCount;
}

aditof::Status AdiTofDemoController::enableNoiseReduction(bool en) {
    using namespace aditof;

    if (m_cameraInUse < 0)
        return Status::GENERIC_ERROR;

    if (!en) {
        return m_cameras[static_cast<unsigned int>(m_cameraInUse)]->setControl(
            "noise_reduction_threshold", std::to_string(0));
    }

    return Status::OK;
}

aditof::Status
AdiTofDemoController::setNoiseReductionThreshold(uint16_t threshold) {
    using namespace aditof;

    if (m_cameraInUse < 0)
        return Status::GENERIC_ERROR;

    return m_cameras[static_cast<unsigned int>(m_cameraInUse)]->setControl(
        "noise_reduction_threshold", std::to_string(threshold));
}

aditof::Status AdiTofDemoController::setIrGammaCorrection(float gamma) {
    using namespace aditof;

    if (m_cameraInUse < 0)
        return Status::GENERIC_ERROR;

    return m_cameras[static_cast<unsigned int>(m_cameraInUse)]->setControl(
        "ir_gamma_correction", std::to_string(gamma));
}

aditof::Status
AdiTofDemoController::setCameraRevision(const std::string &revision) {
    using namespace aditof;

    if (m_cameraInUse < 0)
        return Status::GENERIC_ERROR;

    return m_cameras[static_cast<unsigned int>(m_cameraInUse)]->setControl(
        "revision", revision);
}
