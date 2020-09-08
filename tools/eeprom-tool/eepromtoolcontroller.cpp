#include "eepromtoolcontroller.h"

#include <aditof/device_construction_data.h>
#include <aditof/device_enumerator_interface.h>
#include <aditof/device_enumerator_factory.h>
#include <aditof/device_factory.h>
#include <aditof/eeprom_factory.h>

#include <algorithm>
#include <glog/logging.h>
#include <iostream>
#include <fstream>
#include <ios>

EepromToolController::EepromToolController(){
  //TODO
}
static const std::string skEepromName = "24c1024";
#define EEPROM_SIZE 131072


aditof::Status EepromToolController::setConnection(aditof::ConnectionType connectionType, const std::string& ip) {
    const unsigned int usedDevData = 0;

    aditof::Status status;
    std::unique_ptr<aditof::DeviceEnumeratorInterface> enumerator;
    std::vector<aditof::DeviceConstructionData> devicesData;
    aditof::DeviceConstructionData devData;

    void * handle;

    if (connectionType == aditof::ConnectionType::ETHERNET){
        enumerator = aditof::DeviceEnumeratorFactory::buildDeviceEnumeratorEthernet(ip);
    }
    else{
        enumerator = aditof::DeviceEnumeratorFactory::buildDeviceEnumerator();
    }

    enumerator->findDevices(devicesData);
    //TO DO add check on devices size
    devData = devicesData[usedDevData];
    m_device = aditof::DeviceFactory::buildDevice(devData);
    m_device->open();
    m_device->getHandle(&handle);
    
    auto iter = std::find_if(devData.eeproms.begin(), devData.eeproms.end(),
                             [](const aditof::EepromConstructionData &eData) {
                                 return eData.driverName == skEepromName;
                             });
    if (iter == devData.eeproms.end()) {
        LOG(ERROR)
            << "No available info about the EEPROM required by the camera";
        return aditof::Status::INVALID_ARGUMENT; //TODO review returned status for this situation 
    }

    m_eeprom = aditof::EepromFactory::buildEeprom(devData.connectionType);
    if (!m_eeprom) {
        LOG(ERROR) << "Failed to create an Eeprom object";
        return aditof::Status::INVALID_ARGUMENT;//TODO review returned status for this situation 
    }

    const aditof::EepromConstructionData &eeprom24c1024Info = *iter;
    status = m_eeprom->open(handle, eeprom24c1024Info.driverName.c_str(),
                            eeprom24c1024Info.driverPath.c_str());
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to open EEPROM with name "
                   << devData.eeproms.back().driverName << " is available";
        return status;
    }

    return aditof::Status::OK;
}

aditof::Status EepromToolController::writeEeprom(const std::vector<uint8_t> data){
    float size = static_cast<float>(data.size());

    m_eeprom->write((uint32_t)0, (uint8_t *)&size, (size_t)4);
    m_eeprom->write((uint32_t)4, (uint8_t *)data.data(), (size_t)size);
    return aditof::Status::OK;
}

aditof::Status EepromToolController::readEeprom(std::vector<uint8_t>& data){
    float read_size = 100;
    aditof::Status status;

    m_eeprom->write(EEPROM_SIZE - 5, (uint8_t *)&read_size, 4);
    
    m_eeprom->read((uint32_t)0, (uint8_t *)&read_size, (size_t)4);
    LOG(INFO) << "EEPROM calibration data size " << read_size << " bytes";

    if (read_size > EEPROM_SIZE) {
        LOG(WARNING) << "Invalid calibration data size";
        return aditof::Status::GENERIC_ERROR;
    }
    data.resize(read_size);

    printf("calibration data size %d\n", (uint32_t)read_size);
    status = m_eeprom->read(4, data.data(), read_size);
    if (status != aditof::Status::OK) {
        data.resize(0);
        LOG(WARNING) << "Failed to read from eeprom";
        return status;
    }

    return aditof::Status::OK;
}
//TODO make static
 aditof::Status EepromToolController::readFile(char const* filename, std::vector<uint8_t>& data){
    std::ifstream ifs(filename, std::ios::binary|std::ios::ate);
    std::ifstream::pos_type pos = ifs.tellg();

    data.resize(pos);

    ifs.seekg(0, std::ios::beg);
    ifs.read((char*)&data[0], pos);

    return aditof::Status::OK;
}

 aditof::Status EepromToolController::writeFile(char const* filename, const std::vector<uint8_t> data){
  
    auto myfile = std::fstream(filename, std::ios::out | std::ios::binary);
    myfile.write((char*)&data[0], data.size());
    myfile.close();
 
   return aditof::Status::OK;
}

 aditof::Status EepromToolController::writeFileToEeprom(char const* filename){
    std::vector<uint8_t> data;
    aditof::Status status;

    status = readFile(filename, data);
    if (status != aditof::Status::OK){
        LOG(ERROR) << "Failed to read data from file";
        return status;
    }

    status = writeEeprom(data);
    if (status != aditof::Status::OK){
        LOG(ERROR) << "Failed to write data to EEPROM";
        return status;
    }

    return aditof::Status::OK;
 }

 aditof::Status EepromToolController::readEepromToFile(char const* filename){
    std::vector<uint8_t> data;
    aditof::Status status;

    status = readEeprom(data);
    if (status != aditof::Status::OK){
        LOG(ERROR) << "Failed to read data from EEPROM";
        return status;
    }

    status = writeFile(filename, data);
    if (status != aditof::Status::OK){
        LOG(ERROR) << "Failed to write data to file";
        return status;
    }

    return aditof::Status::OK;
 }


EepromToolController::~EepromToolController() {
    m_eeprom->close();
    m_device->stop();
    delete m_system;
}

