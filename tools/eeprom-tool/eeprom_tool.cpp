/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Analog Devices, Inc.
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
#include "eeprom_tool.h"

#include "camera_eeprom_factory.h"
#include "camera_eeprom_interface.h"
#include <aditof/connections.h>
#include <aditof/sensor_enumerator_factory.h>
#include <aditof/sensor_enumerator_interface.h>

#include <algorithm>
#include <fstream>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <ios>
#include <iostream>

const std::string connectionTypeMapStr[] = {"ON_TARGET", "USB", "NETWORK"};

EepromTool::EepromTool() : m_storageOpened(false) {
    //TODO ??
}

aditof::Status EepromTool::setConnection(aditof::ConnectionType connectionType,
                                         std::string ip,
                                         std::string eepromName) {
    const unsigned int usedDepthSensorIndex = 0;
    void *handle = nullptr;
    aditof::Status status;
    std::unique_ptr<aditof::SensorEnumeratorInterface> enumerator;
    std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;

    //create enumerator based on specified connection type
    switch (connectionType) {
    case aditof::ConnectionType::NETWORK:
        enumerator =
            aditof::SensorEnumeratorFactory::buildNetworkSensorEnumerator(ip);
        if (enumerator == nullptr) {
            LOG(ERROR) << "Network is not enabled";
            return aditof::Status::INVALID_ARGUMENT;
        }
        break;
    case aditof::ConnectionType::USB:
        enumerator =
            aditof::SensorEnumeratorFactory::buildUsbSensorEnumerator();
        break;
    case aditof::ConnectionType::ON_TARGET:
        enumerator =
            aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
        break;
    default:
        LOG(ERROR) << "Selected connection type is not supported";
        return aditof::Status::INVALID_ARGUMENT;
        break;
    }

    LOG(INFO)
        << "Setting connection via "
        << connectionTypeMapStr[static_cast<unsigned int>(connectionType)];

    enumerator->searchSensors();
    enumerator->getStorages(m_storages);
    enumerator->getDepthSensors(depthSensors);

    if (depthSensors.size() == 0) {
        LOG(ERROR) << "Cannot find any camera depth sensor";
        return aditof::Status::GENERIC_ERROR;
    }

    if (usedDepthSensorIndex >= depthSensors.size()) {
        LOG(ERROR) << "Depth sensor index to use is out of bounds";
        return aditof::Status::GENERIC_ERROR;
    }

    m_depthSensor = depthSensors[usedDepthSensorIndex];

    if (m_storages.size() == 0) {
        LOG(ERROR) << "Cannot find any storages";
        return aditof::Status::GENERIC_ERROR;
    }

    if (eepromName.empty()) {
        if (m_storages.size() > 1) {
            LOG(ERROR) << "Multiple storages available but none selected.";
            return aditof::Status::INVALID_ARGUMENT;
        }
        if (m_storages.size() == 1) {
            m_storages[0]->getName(eepromName);
        }
    }

    //get eeproms with the specified name
    auto iter = std::find_if(
        m_storages.begin(), m_storages.end(),
        [eepromName](
            const std::shared_ptr<aditof::StorageInterface>
                &storage) { //TODO make storage const when getName is const
            std::string storageName;
            storage->getName(storageName);
            return storageName == eepromName;
        });
    if (iter == m_storages.end()) {
        LOG(ERROR) << "No available info about the EEPROM required by the user";
        return aditof::Status::INVALID_ARGUMENT; //TODO review returned status
    }

    m_storage = *iter;

    //get handle
    status = m_depthSensor->open();
    if (status != aditof::Status::OK) {
        LOG(WARNING) << "Failed to open depth sensor";
        return status;
    }

    status = m_depthSensor->getHandle(&handle);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to obtain the handle";
        return status;
    }

    //open eeprom
    status = m_storage->open(handle);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to open storage";
        return status;
    }
    m_storageOpened = true;

    m_camera_eeprom = CameraEepromFactory::buildEeprom(m_storage);

    LOG(INFO) << "Successfully created connection to EEPROM";

    return aditof::Status::OK;
}

aditof::Status EepromTool::writeFileToEeprom(char const *filename) {
    std::vector<uint8_t> data;
    aditof::Status status;

    status = readFile(filename, data);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read data from file";
        return status;
    }

    status = writeEeprom(data);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to write data to EEPROM";
        return status;
    }

    LOG(INFO) << "Successfully wrote data to EEPROM from file";

    return aditof::Status::OK;
}

aditof::Status EepromTool::listEeproms() {
    printf("found %ld storage%s:\n", m_storages.size(),
           m_storages.size() == 1 ? "" : "s");

    //list all found eeproms that are contained in the map
    for (std::shared_ptr<aditof::StorageInterface> storage : m_storages) {
        std::string storageName;
        storage->getName(storageName);
        if (EEPROMS.count(storageName)) {
            printf("%s\n", storageName.c_str());
        } else {
            LOG(WARNING) << "Unknown storage found " << storageName;
        }
    }

    return aditof::Status::OK;
}

aditof::Status EepromTool::readEepromToFile(char const *filename) {
    std::vector<uint8_t> data;
    aditof::Status status;

    status = readEeprom(data);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read data from EEPROM";
        return status;
    }

    status = writeFile(filename, data);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to write data to file";
        return status;
    }

    LOG(INFO) << "Successfully wrote data to file from EEPROM";

    return aditof::Status::OK;
}

aditof::Status EepromTool::writeEeprom(const std::vector<uint8_t> data) {
    return m_camera_eeprom->write(data);
}

aditof::Status EepromTool::readEeprom(std::vector<uint8_t> &data) {
    return m_camera_eeprom->read(data);
}

aditof::Status EepromTool::readFile(char const *filename,
                                    std::vector<uint8_t> &data) {
    std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
    std::ifstream::pos_type pos = ifs.tellg();

    data.resize(pos);

    ifs.seekg(0, std::ios::beg);
    ifs.read((char *)&data[0], pos);

    LOG(INFO) << "Successfully read data from file";

    return aditof::Status::OK;
}

aditof::Status EepromTool::writeFile(char const *filename,
                                     const std::vector<uint8_t> data) {
    std::fstream myfile(filename, std::ios::out | std::ios::binary);

    myfile.write((char *)&data[0], data.size());
    myfile.close();

    LOG(INFO) << "Successfully wrote data to file";

    return aditof::Status::OK;
}

EepromTool::~EepromTool() {
    if (m_storageOpened) {
        m_storage->close();
    }

    LOG(INFO) << "Destroyed connection";
}
