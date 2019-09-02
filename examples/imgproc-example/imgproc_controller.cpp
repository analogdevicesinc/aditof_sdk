#include "imgproc_controller.h"

#include <glog/logging.h>
#include <iostream>

ImgProc_Controller::ImgProc_Controller()
    : m_cameraInUse(-1), m_frameRequested(false) {

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

ImgProc_Controller::~ImgProc_Controller() {
    if (m_cameraInUse == -1) {
        return;
    }
    stopCapture();
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->stop();
}

void ImgProc_Controller::startCapture() {
    if (m_cameraInUse == -1) {
        return;
    }

    m_stopFlag = false;
    m_workerThread =
        std::thread(std::bind(&ImgProc_Controller::captureFrames, this));
}

void ImgProc_Controller::stopCapture() {
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

std::string ImgProc_Controller::getMode() const {
    // TODO: implement get mode
    return "";
}

void ImgProc_Controller::setMode(const std::string &mode) {
    if (m_cameraInUse == -1) {
        return;
    }
    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
    camera->setMode(mode);
}

aditof::Status ImgProc_Controller::writeAFEregister(uint16_t * /*address*/,
                                                    uint16_t * /*data*/,
                                                    uint16_t /*noOfEntries*/) {
    // TODO: implement write afe regs
    return aditof::Status::OK;
}

aditof::Status ImgProc_Controller::readAFEregister(uint16_t * /*address*/,
                                                   uint16_t * /*data*/,
                                                   uint16_t /*noOfEntries*/) {
    // TODO: implement read afe regs
    return aditof::Status::OK;
}

std::shared_ptr<aditof::Frame> ImgProc_Controller::getFrame() {
    return m_queue.dequeue();
}

void ImgProc_Controller::requestFrame() {
    std::unique_lock<std::mutex> lock(m_requestMutex);
    m_frameRequested = true;
    lock.unlock();
    m_requestCv.notify_one();
}

bool ImgProc_Controller::hasCamera() const { return !m_cameras.empty(); }

void ImgProc_Controller::captureFrames() {
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

        m_queue.enqueue(frame);
        m_frameRequested = false;
    }
}
