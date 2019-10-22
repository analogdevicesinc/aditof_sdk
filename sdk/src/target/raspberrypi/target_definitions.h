#ifndef TARGET_DEFINITIONS_H
#define TARGET_DEFINITIONS_H

#ifdef REVB
static const char *TEMP_SENSOR_DEV_PATH = "/dev/i2c-1";
static const char *EEPROM_DEV_PATH = "/sys/bus/i2c/devices/1-0056/eeprom";
#else
static const char *TEMP_SENSOR_DEV_PATH = "/dev/i2c-0";
static const char *EEPROM_DEV_PATH = "/sys/bus/i2c/devices/0-0056/eeprom";
#endif // REVB

static const char *CAPTURE_DEVICE_NAME = "unicam";

#endif // TARGET_DEFINITIONS_H
