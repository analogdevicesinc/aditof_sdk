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
#include "eeprom.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int eeprom_open(const char *dev_fqn, eeprom *e) {
    e->valid = 0;
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
    e->valid = 1;

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

int eeprom_write_buf(eeprom *e, unsigned int addr, unsigned char *buf,
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
        e->valid = 0;
    }
    return 0;
}
