#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include "i2c_common.h"

#define TEMPERATURE_MSB_REG 0x0
#define TEMPERATURE_LSB_REG 0x1
#define CONFIGURATION_REG 0x3

/*
 * Open the temperature sensor device
 */
int temp_sensor_open(const char *dev_fqn, int addr, temp_sensor *t);
/*
 * read the temperature sensor data
 */
int temp_sensor_read(temp_sensor *t, float *temp_val);
/*
 * read the temperature sensor data of size 1 byte
 */
int temp_read_byte_data(temp_sensor *t, __u16 addr_reg);
/*
 * close the teperature sensor device
 */
int temp_sensor_close(temp_sensor *t);
/*
 * write the 1 byte daat to temperature sensor device
 */
int temp_write_byte(temp_sensor *t, __u16 mem_addr, __u8 data);

#endif /* TEMP_SENSOR_H */
