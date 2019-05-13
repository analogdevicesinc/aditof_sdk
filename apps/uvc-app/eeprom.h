#ifndef EEPROM_H
#define EEPROM_H

typedef struct eeprom_st
{
	char *dev;
	unsigned int length;
	FILE *fd;
}eeprom;

/*
 * opens the eeprom device at [dev_fqn] (i.e. /dev/i2c-N) whose address is
 * [addr] and set the eeprom_24c32 [e]
 */
int eeprom_open(char *dev_fqn, eeprom*);
/*
 * closees the eeprom device [e] 
 */
int eeprom_close(eeprom *e);
/* 
 * write the data stored in buff to eeprom
 */
int eeprom_write_buf(eeprom *e, unsigned int addr, unsigned char *buf, int size);
/* 
 * read the data from eeprom and store it in buff
 */
int eeprom_read_buf(eeprom *e, unsigned int addr, unsigned char *buf, int size);

#endif /* EEPROM_H */
