#ifndef CAM96TOF1_EEPROM_H
#define CAM96TOF1_EEPROM_H

#include <aditof/system.h>
#include <aditof/eeprom_interface.h>
#include "camera_eeprom_interface.h"
#include <glog/logging.h>

#include <memory>


class Camera96Tof1Eeprom: public CameraEepromInterface {
    public:
        Camera96Tof1Eeprom(std::shared_ptr<aditof::EepromInterface> _eeprom);
        virtual aditof::Status read(std::vector<uint8_t>& data) override;
        virtual aditof::Status write(std::vector<uint8_t> data) override;
    private:
        std::shared_ptr<aditof::EepromInterface> m_eeprom;
};

#endif