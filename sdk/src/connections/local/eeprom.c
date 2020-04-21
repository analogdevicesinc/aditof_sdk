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

static int eeprom_open(const char *dev_fqn, eeprom *e) {
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

static int eeprom_read_buf(eeprom *e, unsigned int addr, unsigned char *buf,
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

static int eeprom_write_buf(eeprom *e, unsigned int addr, unsigned char *buf,
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

static int eeprom_close(eeprom *e) {
    if (e) {
        fclose(e->fd);
        e->fd = NULL;
        e->valid = 0;
    }
    return 0;
}

Eeprom::Eeprom(const char *dev_fqn) 
	: m_path(dev_fqn) {

}

Eeprom::~Eeprom() {
	close();
}

aditof::Status Eeprom::open() {
	using namespace aditof;
	Status status = Status::OK;
	
	if (eeprom_open(m_path.c_str(), &m_edev) < 0) {
        status = Status::GENERIC_ERROR;
    }
	
	return status;
}

aditof::Status Eeprom::close() {
	using namespace aditof;
	
	eeprom_close(&m_edev);
	
	return Status::OK;
}

aditof::Status Eeprom::read(uint32_t address, uint8_t *data,
							size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    if (!m_edev.valid) {
        LOG(WARNING) << "EEPROM not available!";
        return Status::GENERIC_ERROR;
    }

    int ret = eeprom_read_buf(&m_edev, address, data, length);
    if (ret == -1) {
        LOG(WARNING) << "EEPROM read error";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status Eeprom::write(uint32_t address, const uint8_t *data,
                             size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    if (!m_edev.valid) {
        LOG(WARNING) << "EEPROM not available!";
        return Status::GENERIC_ERROR;
    }

    int ret = eeprom_write_buf(&m_edev, address,
                               const_cast<uint8_t *>(data), length);
    if (ret == -1) {
        LOG(WARNING) << "EEPROM write error";
        return Status::GENERIC_ERROR;
    }

    return status;
}