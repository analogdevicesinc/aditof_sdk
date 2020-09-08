#include "cam96tof1_eeprom.h"
#include "eeprom_list.h"

aditof::Status Camera96Tof1Eeprom::read(std::vector<uint8_t>& data){
    float read_size = 100;
    uint32_t eepromSize; 
    aditof::Status status;
    std::string eepromName;

    m_eeprom->getName(eepromName);
    eepromSize = EEPROMS.at(eepromName).size;

    //mistery
    m_eeprom->write(eepromSize - 5, (uint8_t *)&read_size, 4);

    //read size
    m_eeprom->read((uint32_t)0, (uint8_t *)&read_size, (size_t)4);
    LOG(INFO) << "EEPROM calibration data size " << read_size << " bytes";
    if (read_size > eepromSize) {
        LOG(WARNING) << "Invalid calibration data size";
        return aditof::Status::GENERIC_ERROR;
    }

    //read data
    data.resize(read_size);
    status = m_eeprom->read(4, data.data(), read_size);
    if (status != aditof::Status::OK) {
        data.resize(0);
        LOG(WARNING) << "Failed to read from eeprom";
        return status;
    }

    return aditof::Status::OK;
}

aditof::Status Camera96Tof1Eeprom::write(std::vector<uint8_t> data){
    aditof::Status status;
    float size = static_cast<float>(data.size());

    status = m_eeprom->write((uint32_t)0, (uint8_t *)&size, (size_t)4);
    if (status != aditof::Status::OK){
        LOG(ERROR) << "failed to write to eeprom";
        return status;
    }

    status = m_eeprom->write((uint32_t)4, (uint8_t *)data.data(), (size_t)size);
    if (status != aditof::Status::OK){
        LOG(ERROR) << "failed to write to eeprom";
        return status;
    }

    return aditof::Status::OK;
}