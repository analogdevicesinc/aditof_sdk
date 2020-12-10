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
#ifndef STORAGE_INTERFACE_H
#define STORAGE_INTERFACE_H

#include <aditof/status_definitions.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace aditof {

/**
 * @class StorageInterface
 * @brief Interface for storage of data.
 * For example: EEPROM, Flash, file on disk, etc.
 */
class StorageInterface {
  public:
    /**
   * @brief Destructor
   */
    virtual ~StorageInterface() = default;

    /**
     * @brief Open the communication channel with the storage.
     * @param handle - A handle to the object through which communication is done
     * @return Status
     */
    virtual aditof::Status open(void *handle) = 0;

    /**
     * @brief Read data from storage
     * @param address - The address from where the data should be read
     * @param[out] data - The location where the read data should be stored
     * @param bytesCount - The number of bytes to read
     * @return Status
     */
    virtual aditof::Status read(const uint32_t address, uint8_t *data,
                                const size_t bytesCount) = 0;

    /**
     * @brief Write data to storage
     * @param address - The starting address where the data should be written
     * @param data - The location of the data to be written
     * @param bytesCount - The number of bytes to write
     * @return Status
     */
    virtual aditof::Status write(const uint32_t address, const uint8_t *data,
                                 const size_t bytesCount) = 0;

    /**
     * @brief Close the communication channel with the storage.
     * @return Status
     */
    virtual aditof::Status close() = 0;

    /**
     * @brief Retrieves the name of the storage
     * @param[out] name - This gets set with the name of the storage
     * @return Status
     */
    virtual aditof::Status getName(std::string &name) const = 0;
};

} // namespace aditof

#endif // STORAGE_INTERFACE_H
