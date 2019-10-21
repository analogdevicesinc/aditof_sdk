#include "aditofdemocontroller.h"

#include <glog/logging.h>
#include <iostream>

AdiTofDemoController::AdiTofDemoController()
    : m_cameraInUse(-1), m_frameRequested(false),
      m_recorder(new AditofDemoRecorder()) {

    m_system.initialize();
    m_system.getCameraList(m_cameras);
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

AdiTofDemoController::~AdiTofDemoController() {
    if (m_cameraInUse == -1) {
        return;
    }
    stopCapture();
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->stop();
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

    aditof::DeviceInterface *device = camera->getDevice();

    device->readAfeTemp(returnValue.first);
    device->readLaserTemp(returnValue.second);

    return returnValue;
}

aditof::Status AdiTofDemoController::writeAFEregister(uint16_t *address,
                                                      uint16_t *data,
                                                      uint16_t noOfEntries) {
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
                                          unsigned int height,
                                          unsigned int width,
                                          unsigned int fps) {
    m_recorder->startRecording(fileName, height, width, fps);
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

int AdiTofDemoController::getRange() const {
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
        cameraDetails);
    return cameraDetails.range;
}
