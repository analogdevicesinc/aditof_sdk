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
#ifndef USB_LINUX_UTILS_H
#define USB_LINUX_UTILS_H

#include <cstdint>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define MAX_PACKET_SIZE (58)
#define MAX_BUF_SIZE (MAX_PACKET_SIZE + 2)

class UsbLinuxUtils {
  public:
    static int xioctl(int fh, unsigned long request, void *arg);

    /**
     * @brief uvcExUnitReadOnePacket - Reads one packet of data through the UVC Extension Unit
     * @param fd - The file descriptor for the UVC communication channel
     * @param selector - The UVC control selector
     * @param address - The address from where the reading should start
     * @param[out] data - The location where the read data should be stored
     * @param nbPacketBytes - The size the package should have
     * @param nbBytesToRead - The number of bytes to read from the entire package
     * @param getOnly - Use only the UVC_GET_CUR property. Skip setting the address with UVC_SET_CUR
     * @return int - Return 0 is operation is succesful or a negative error code otherwise
     */
    static int uvcExUnitReadOnePacket(int fd, uint8_t selector,
                                      uint32_t address, uint8_t *data,
                                      uint8_t nbPacketBytes,
                                      uint8_t nbBytesToRead,
                                      bool getOnly = false);

    /**
     * @brief uvcExUnitReadBuffer - Reads a chunk of data (a buffer) through the UVC Extension Unit
     * @param fd - The file descriptor for the UVC communication channel
     * @param selector - The UVC control selector
     * @param address - The address from there the reading should start
     * @param data - The location where the read data should be stored
     * @param bufferLength - The number of bytes to read
     * @return int - Return 0 is operation is succesful or a negative error code otherwise
     */
    static int uvcExUnitReadBuffer(int fd, uint8_t selector, uint32_t address,
                                   uint8_t *data, uint32_t bufferLength);

    /**
     * @brief uvcExUnitWriteBuffer
     * @param fd - The file descriptor for the UVC communication channel
     * @param selector - The UVC control selector
     * @param address - The address where the writing should start
     * @param data - The location of the data to be written
     * @param bufferLength - The number of bytes to write
     * @return int - Return 0 is operation is succesful or a negative error code otherwise
     */
    static int uvcExUnitWriteBuffer(int fd, uint8_t selector, uint32_t address,
                                    const uint8_t *data, uint32_t bufferLength);
};

#endif // USB_LINUX_UTILS_H
