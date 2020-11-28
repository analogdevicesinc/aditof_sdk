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
#ifndef USB_UTILS_H
#define USB_UTILS_H

#include <string>
#include <vector>

class UsbUtils {
  public:
    /**
     * @brief Parses sensor tokens which are strings containing a key-value pair and finds
     * the type of the sensor which can take values starting from 0.
     * @param tokens - The tokens to be parsed.
     * @return int
     */
    static int getDepthSensoType(const std::vector<std::string> &tokens);

    /**
     * @brief Parses sensor tokens which are strings containing a key-value pair
     * (e.g. "EEPROM_NAME=eeprom1") and returns the names of the storages.
     * @param tokens - The tokens to be parsed.
     * return std::vector<std::string>
     */
    static std::vector<std::string>
    getStorageNames(const std::vector<std::string> &tokens);

    /**
     * @brief Parses sensor tokens which are strings containing a key-value pair
     * and returns the names of the temperature sensors.
     * @param tokens - The tokens to be parsed.
     * return std::vector<std::string>
     */
    static std::vector<std::string>
    getTemperatureSensorNames(const std::vector<std::string> &tokens);
};

#endif // USB_UTILS_H
