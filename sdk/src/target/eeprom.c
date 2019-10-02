#include "eeprom.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int eeprom_open(const char *dev_fqn, eeprom *e) {
    e->fd = fopen(dev_fqn, "w+");
    if (e->fd == NULL) {
        fprintf(stderr, "Error eeprom_open: %s\n", strerror(errno));
        return -1;
    }

    fseek(e->fd, 0x0, SEEK_END);
    long len = ftell(e->fd);
    if (len < 0) {
        fprintf(stderr, "Error eeprom_open: %s\n", strerror(errno));
        return -1;
    }
    e->length = (unsigned int)len;
    fseek(e->fd, 0x0, SEEK_SET);

    return 0;
}

int eeprom_read_buf(eeprom *e, unsigned int addr, unsigned char *buf,
                    size_t size) {
    fseek(e->fd, addr, SEEK_SET);
    size_t ret = fread(buf, 1, size, e->fd);
    if (ret < size) {
        fprintf(stderr, "eeprom_read_buf failed with %d: %s\n", errno,
                strerror(errno));
        return -1;
    }
    return 0;
}

int eeprom_write_buf(eeprom *e, unsigned int addr, const unsigned char *buf,
                     size_t size) {
    fseek(e->fd, addr, SEEK_SET);
    size_t ret = fwrite(buf, 1, size, e->fd);
    if (ret < size) {
        fprintf(stderr, "eeprom_write_buf failed with %d: %s\n", errno,
                strerror(errno));
        return -1;
    }
    return 0;
}

int eeprom_close(eeprom *e) {
    if (e) {
        fclose(e->fd);
        e->fd = NULL;
    }
    return 0;
}
