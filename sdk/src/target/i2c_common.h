#ifndef I2C_COMMON_H
#define I2C_COMMON_H

#include <linux/fs.h>
#include <linux/types.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

/*
 * Data for SMBus Messages
 */
#define I2C_SMBUS_BLOCK_MAX 32 /* As specified in SMBus standard */

union i2c_smbus_data {
    __u8 byte;
    __u16 word;
    __u8 block[I2C_SMBUS_BLOCK_MAX + 3]; /* block[0] is used for length */
    /* one more for read length in block process call */
    /* and one more for PEC */
};

/* smbus_access read or write markers */
#define I2C_SMBUS_READ 1
#define I2C_SMBUS_WRITE 0

/* SMBus transaction types (size parameter in the above functions)
   Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_BYTE 1
#define I2C_SMBUS_BYTE_DATA 2
#define I2C_SMBUS_WORD_DATA 3

/* this is for i2c-dev.c	*/
#define I2C_SLAVE 0x0703 /* Change slave address			*/
/* Attn.: Slave address is 7 or 10 bits */

#define I2C_FUNCS 0x0705 /* Get the adapter functionality */

#define I2C_SMBUS 0x0720 /* SMBus-level access */

typedef struct i2c_dev {
    const char *dev; // device file i.e. /dev/i2c-N
    int addr;        // i2c address
    int fd;          // file descriptor
} temp_sensor;

typedef struct eeprom {
    char *dev;
    unsigned int length;
    FILE *fd;
    int valid;
} eeprom;

/* This is the structure as used in the I2C_SMBUS ioctl call */
struct i2c_smbus_ioctl_data {
    char read_write;
    __u8 command;
    int size;
    union i2c_smbus_data *data;
};

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                     int size, union i2c_smbus_data *data) {
    struct i2c_smbus_ioctl_data args;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;
    return ioctl(file, I2C_SMBUS, &args);
}

static inline __s32 i2c_smbus_read_byte(int file) {
    union i2c_smbus_data data;
    if (i2c_smbus_access(file, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data))
        return -1;
    else
        return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command) {
    union i2c_smbus_data data;
    if (i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA,
                         &data))
        return -1;
    else
        return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command,
                                              __u8 value) {
    union i2c_smbus_data data;
    data.byte = value;
    return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA,
                            &data);
}

static inline __s32 i2c_smbus_write_word_data(int file, __u8 command,
                                              __u16 value) {
    union i2c_smbus_data data;
    data.word = value;
    return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA,
                            &data);
}

#endif /* I2C_COMMON_H */
