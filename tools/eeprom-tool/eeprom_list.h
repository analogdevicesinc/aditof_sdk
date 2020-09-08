#ifndef EEPROMLIST_H
#define EEPROMLIST_H

#include <cstdint>
#include <string>
#include <map>

struct EepromProperties{
    uint16_t size;
    EepromProperties(uint16_t _size):
        size(size)
        {}
};

const std::map<std::string, EepromProperties> EEPROMS = {
    {"24c1024", EepromProperties(131072)},
    {"demo", EepromProperties(42)}
};

#endif