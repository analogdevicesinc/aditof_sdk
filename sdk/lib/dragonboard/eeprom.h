#ifndef EEPROM_H
#define EEPROM_H

#include "i2c_common.h"

/*
 * opens the eeprom device with filesystem access
 */
int eeprom_open(const char *dev_fqn, eeprom *e);
/*
 * closes the eeprom device
 */
int eeprom_close(eeprom *e);
/*
 * write the data stored in buff to eeprom
 */
int eeprom_write_buf(eeprom *e, unsigned int addr, unsigned char *buf,
                     size_t size);
/*
 * read the data from eeprom and store it in buff
 */
int eeprom_read_buf(eeprom *e, unsigned int addr, unsigned char *buf,
                    size_t size);

#endif /* EEPROM_H */
