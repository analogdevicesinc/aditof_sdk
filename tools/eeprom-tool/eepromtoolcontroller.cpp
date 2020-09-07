#include "eepromtoolcontroller.h"

#include <aditof/device_construction_data.h>
#include <aditof/device_enumerator_interface.h>
#include <aditof/device_enumerator_factory.h>
#include <aditof/device_factory.h>
#include <aditof/eeprom_factory.h>

#include <algorithm>
#include <glog/logging.h>
#include <iostream>

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

EepromToolController::~EepromToolController() {
    //TODO
    delete m_system;
}

