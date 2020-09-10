#ifndef EEPROMLIST_H
#define EEPROMLIST_H

#include <cstdint>
#include <string>
#include <map>

struct EepromProperties{
    uint32_t size;
    EepromProperties(uint32_t _size):
        size(_size)
        {}
};

const std::map<std::string, EepromProperties> EEPROMS = {
    {"24c1024", EepromProperties((uint32_t)131072)},
    {"demo", EepromProperties((uint32_t)42)}
};

#endif