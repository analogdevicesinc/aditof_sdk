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
#include <aditof/device_enumerator_factory.h>
#include <aditof/device_enumerator_interface.h>
#include <aditof/device_factory.h>
#include <aditof/eeprom_construction_data.h>
#include <aditof/eeprom_factory.h>

#include <algorithm>
#include <fstream>
#include <glog/logging.h>
#include <ios>
#include <iostream>

#ifdef CHICONY_006
const aditof::SensorType sensorType = aditof::SensorType::SENSOR_CHICONY;
#else
const aditof::SensorType sensorType = aditof::SensorType::SENSOR_96TOF1;
#endif

const std::string connectionTypeMapStr[] = {"LOCAL", "USB", "ETHERNET"};

EepromTool::EepromTool() {
    //TODO ??
}

aditof::Status EepromTool::setConnection(aditof::ConnectionType connectionType,
                                         std::string ip,
                                         std::string eepromName) {
    const unsigned int usedDevDataIndex = 0;
    void *handle = nullptr;
    aditof::Status status;
    std::unique_ptr<aditof::DeviceEnumeratorInterface> enumerator;
    std::vector<aditof::DeviceConstructionData> devicesData;

    //create enumerator based on specified connection type
    if (connectionType == aditof::ConnectionType::ETHERNET) {
        enumerator =
            aditof::DeviceEnumeratorFactory::buildDeviceEnumeratorEthernet(ip);
        if (enumerator == nullptr) {
            LOG(ERROR) << "Network is not enabled";
            return aditof::Status::INVALID_ARGUMENT;
        }
    } else {
        enumerator = aditof::DeviceEnumeratorFactory::buildDeviceEnumerator();
    }

    LOG(INFO)
        << "Setting connection via "
        << connectionTypeMapStr[static_cast<unsigned int>(connectionType)];

    //get devices
    enumerator->findDevices(devicesData);
    //only keep devices that use the specified connection type
    devicesData.erase(
        std::remove_if(devicesData.begin(), devicesData.end(),
                       [connectionType](aditof::DeviceConstructionData dev) {
                           return dev.connectionType != connectionType;
                       }),
        devicesData.end());

    if (devicesData.size() <= usedDevDataIndex) {
        LOG(ERROR) << "Cannot find device at index " << usedDevDataIndex;
        return aditof::Status::GENERIC_ERROR;
    }
    m_devData = devicesData[usedDevDataIndex];

    if (eepromName.empty()) {
        if (m_devData.eeproms.size() > 1) {
            LOG(ERROR) << "Multiple EEPROMs available but none selected.";
            return aditof::Status::INVALID_ARGUMENT;
        }
        if (m_devData.eeproms.size() == 1) {
            eepromName = m_devData.eeproms[0].driverName;
        }
    }

    //get eeproms with the specified name
    auto iter =
        std::find_if(m_devData.eeproms.begin(), m_devData.eeproms.end(),
                     [eepromName](const aditof::EepromConstructionData &eData) {
                         return eData.driverName == eepromName;
                     });
    if (iter == m_devData.eeproms.end()) {
        LOG(ERROR) << "No available info about the EEPROM required by the user";
        return aditof::Status::INVALID_ARGUMENT; //TODO review returned status
    }

    m_eeprom = aditof::EepromFactory::buildEeprom(m_devData.connectionType);
    if (!m_eeprom) {
        LOG(ERROR) << "Failed to create an Eeprom object";
        return aditof::Status::INVALID_ARGUMENT; //TODO review returned status
    }

    //get handle
    m_device = aditof::DeviceFactory::buildDevice(m_devData);
    status = m_device->open();
    if (status != aditof::Status::OK) {
        LOG(WARNING) << "Failed to open device";
        return status;
    }

    status = m_device->getHandle(&handle);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to obtain the handle";
        return status;
    }

    //open eeprom
    const aditof::EepromConstructionData &eepromInfo = *iter;
    status = m_eeprom->open(handle, eepromInfo.driverName.c_str(),
                            eepromInfo.driverPath.c_str());
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to open EEPROM with name "
                   << m_devData.eeproms.back().driverName;
        return status;
    }

    m_camera_eeprom = CameraEepromFactory::buildEeprom(sensorType, m_eeprom);

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
    printf("found %ld eeprom%s:\n", m_devData.eeproms.size(),
           m_devData.eeproms.size() == 1 ? "" : "s");

    //list all found eeproms that are contained in the map
    for (aditof::EepromConstructionData eepromData : m_devData.eeproms) {
        if (EEPROMS.count(eepromData.driverName)) {
            printf("%s\n", eepromData.driverName.c_str());
        } else {
            LOG(WARNING) << "Unknown eeprom found " << eepromData.driverName;
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
    if (m_eeprom) {
        m_eeprom->close();
    }
    if (m_device) {
        m_device->stop();
    }
    LOG(INFO) << "Destroyed connection";
}
