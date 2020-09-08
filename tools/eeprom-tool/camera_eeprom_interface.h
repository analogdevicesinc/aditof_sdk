#ifndef CAMERA_EEPROM_INTERFACE_H
#define CAMERA_EEPROM_INTERFACE_H

#include <aditof/system.h>

class CameraEepromInterface {
    public:
        virtual ~CameraEepromInterface() = default;
        virtual aditof::Status read(std::vector<uint8_t>&) = 0;
        virtual aditof::Status write(std::vector<uint8_t>) = 0;
};

#endif
