#include "basic_controller.h"

#include <glog/logging.h>
#include <iostream>

Basic_Controller::Basic_Controller() : m_cameraInUse(-1) {
    aditof::Status status = aditof::Status::OK;

    status = m_system.initialize();
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not initialize system!";
    }

    m_system.getCameraList(m_cameras);
    if (m_cameras.size()) {
        // Use the first camera that is found
        m_cameraInUse = 0;
        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];

        camera->initialize();

        std::vector<std::string> frameTypes;
        camera->getAvailableFrameTypes(frameTypes);
        if (frameTypes.empty()) {
            LOG(WARNING) << "No frame type available!";
            return;
        }
        camera->setFrameType(frameTypes.front());

        std::vector<std::string> modes;
        camera->getAvailableModes(modes);
        if (modes.empty()) {
            LOG(WARNING) << "No camera modes available!";
            return;
        }
        status = camera->setMode(modes.front());
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Could not set camera mode!";
        }

    } else {
        LOG(WARNING) << "No cameras found!";
    }
}

Basic_Controller::~Basic_Controller() {
    if (m_cameraInUse == -1) {
        return;
    }
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->stop();
}

std::vector<aditof::Frame> Basic_Controller::captureFrames() {
    while (!m_stopFlag.load()) {

        if (m_stopFlag) {
            break;
        }

        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
        auto frame = std::make_shared<aditof::Frame>();
        aditof::Status status = camera->requestFrame(frame.get());
        if (status != aditof::Status::OK) {
            continue;
        }

        LOG(ERROR) << "every frame";
        m_frames.push_back(*frame);
        //   m_queue.enqueue(frame);
    }
    return m_frames;
}

aditof::Frame Basic_Controller::getFrame() {
    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
    auto frame = std::make_shared<aditof::Frame>();
    camera->requestFrame(frame.get());
    return *frame;
}

void Basic_Controller::printFrame(aditof::Frame frame) {
    uint16_t *data1;
    aditof::Status status = frame.getData(aditof::FrameDataType::RAW, &data1);

    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not get frame data!";
    }

    if (!data1) {
        LOG(ERROR) << "no memory allocated in frame";
    }

    aditof::FrameDetails fDetails;
    frame.getDetails(fDetails);
    for (unsigned int i = 0; i < fDetails.width * fDetails.height; ++i) {
        std::cout << data1[i] << " ";
    }
}
