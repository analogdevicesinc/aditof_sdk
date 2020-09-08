#ifndef EEPROMLIST_H
#define EEPROMLIST_H

#include <cstdint>
#include <string>
#include <map>

struct EepromProperties{
    uint16_t size;
    EepromProperties(uint16_t _size):
        size(_size)
        {}
};

const std::map<std::string, EepromProperties> EEPROMS = {
    {"24c1024", EepromProperties((uint16_t)131072)},
    {"demo", EepromProperties((uint16_t)42)}
};

#endif