#include "temp_sensor.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int temp_sensor_read(temp_sensor *t, float *temp_val) {
    short temp_reg_val, conf_reg_val;
    int ret;

    /* Read the configuration register*/
    conf_reg_val = temp_read_byte_data(t, CONFIGURATION_REG);
    if (conf_reg_val < 0) {
        return -1;
    }

    /* Read the temperature value MSB register*/
    ret = temp_read_byte_data(t, TEMPERATURE_MSB_REG);
    if (ret < 0) {
        return -1;
    }
    temp_reg_val = ret << 8;

    /* Read the temperature LSB register */
    ret = temp_read_byte_data(t, TEMPERATURE_LSB_REG);
    if (ret < 0) {
        return -1;
    }
    temp_reg_val |= ret;

    /* Compute the temperature in degC */
    if (conf_reg_val & 0x80) {
        if (temp_reg_val & 0x1000) {
            *temp_val = (temp_reg_val - 65536) / 128.0;
        } else {
            *temp_val = temp_reg_val / 128.0;
        }
    } else {
        temp_reg_val = temp_reg_val >> 0x3;
        if (temp_reg_val & 0x1000) {
            *temp_val = (temp_reg_val - 8192) / 16.0;
        } else {
            *temp_val = temp_reg_val / 16.0;
        }
    }

    return 0;
}

int temp_sensor_open(const char *dev_fqn, int addr, temp_sensor *t) {
    int funcs, fd, r;
    t->fd = t->addr = 0;
    t->dev = NULL;

    fd = open(dev_fqn, O_RDWR);
    if (fd <= 0) {
        fprintf(stderr, "Error in temperature sensor open: %s\n",
                strerror(errno));
        return -1;
    }

    // get funcs list
    if ((r = ioctl(fd, I2C_FUNCS, &funcs)) < 0) {
        fprintf(stderr, "Error in temperature sensor open: %s\n",
                strerror(errno));
        return -1;
    }

    // set working device
    if ((r = ioctl(fd, I2C_SLAVE, addr)) < 0) {
        fprintf(stderr, "Error temperature sensor open: %s\n", strerror(errno));
        return -1;
    }
    t->fd = fd;
    t->addr = addr;
    t->dev = dev_fqn;

    return 0;
}

int temp_sensor_close(temp_sensor *t) {
    close(t->fd);
    t->fd = -1;
    t->dev = NULL;

    return 0;
}

int temp_read_byte_data(temp_sensor *t, __u16 addr_reg) {
    int val;
    val = i2c_smbus_read_byte_data(t->fd, addr_reg);

    return val;
}

int temp_write_byte(temp_sensor *t, __u16 mem_addr, __u8 data) {
    int r;
    __u8 buf[2] = {(__u8)(mem_addr & 0x00ff), data};
    r = i2c_smbus_write_byte_data(t->fd, buf[0], buf[1]);
    if (r < 0) {
        fprintf(stderr, "Error i2c_smbus_write_byte_data: %s\n",
                strerror(errno));
    }

    return r;
}
