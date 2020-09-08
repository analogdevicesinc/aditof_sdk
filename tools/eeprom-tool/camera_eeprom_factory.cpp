#include "camera_eeprom_factory.h"
#include "cam96tof1_eeprom.h"

std::unique_ptr<CameraEepromInterface> 
CameraEepromFactory::buildEeprom(aditof::SensorType sensorType, std::shared_ptr<aditof::EepromInterface> eeprom){
    switch (sensorType) {
        case aditof::SensorType::SENSOR_96TOF1:
            return std::unique_ptr<CameraEepromInterface>(new Camera96Tof1Eeprom(eeprom));
        case aditof::SensorType::SENSOR_CHICONY:
            return nullptr;
    }
    return nullptr;
}
