#include "eepromtoolcontroller.h"

#include <glog/logging.h>
#include <iostream>

EepromToolController::EepromToolController()
    : m_cameraInUse(-1){
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

bool EepromToolController::setRegularConnection() {
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

bool EepromToolController::setEthernetConnection(const std::string &ip) {
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

EepromToolController::~EepromToolController() {
    if (m_cameraInUse == -1) {
        return;
    }
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->stop();
    delete m_system;
}


aditof::Status EepromToolController::writeAFEregister(uint16_t *address,
                                                      uint16_t *data,
                                                      uint16_t noOfEntries) {
    if (m_cameraInUse == -1) {
        return aditof::Status::GENERIC_ERROR;
    }

    auto device =
        m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDevice();
    return device->writeAfeRegisters(address, data, noOfEntries);
}

aditof::Status EepromToolController::readAFEregister(uint16_t *address,
                                                     uint16_t *data,
                                                     uint16_t noOfEntries) {

    auto device =
        m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDevice();
    return device->readAfeRegisters(address, data, noOfEntries);
}

bool EepromToolController::hasCamera() const { return !m_cameras.empty(); }
