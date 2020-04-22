/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
