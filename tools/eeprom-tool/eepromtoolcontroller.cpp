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

EepromToolController::EepromToolController()
    : m_eepromInUse(-1){
  //TODO
}
static const std::string skEepromName = "24c1024";

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
    std::shared_ptr<aditof::DeviceInterface> device = aditof::DeviceFactory::buildDevice(devData);
    device->open();
    device->getHandle(&handle);
    
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

aditof::Status EepromToolController::checkData(const std::vector<uint8_t> data){
    return aditof::Status::OK;
}

aditof::Status EepromToolController::writeEeprom(const std::vector<uint8_t> data){
    m_eeprom->write(0, data.data(), data.size());
    return aditof::Status::OK;
}

aditof::Status EepromToolController::readEeprom(std::vector<uint8_t>& data){
    float read_size = 100;
    aditof::Status status;

    m_eeprom->read(0, (uint8_t *)&read_size, 4);
    data.resize(read_size);
    
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


EepromToolController::~EepromToolController() {
    //TODO
    delete m_system;
}

