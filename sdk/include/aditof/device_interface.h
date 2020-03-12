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
#ifndef DEVICE_INTERFACE_H
#define DEVICE_INTERFACE_H

#include <aditof/device_definitions.h>
#include <aditof/frame_definitions.h>
#include <aditof/status_definitions.h>

#include <cstddef>
#include <vector>

namespace aditof {

/**
 * @class DeviceInterface
 * @brief Provides access to the low level functionality of the camera. This
 * includes sensor configuration as well as analog front end(AFE) configuration.
 */
class DeviceInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~DeviceInterface() = default;

    /**
     * @brief Open the communication channels with the hardware.
     * @return Status
     */
    virtual aditof::Status open() = 0;

    /**
     * @brief Start the streaming of data from the device.
     * @return Status
     */
    virtual aditof::Status start() = 0;

    /**
     * @brief Stop the device data stream.
     * @return Status
     */
    virtual aditof::Status stop() = 0;

    /**
     * @brief Return all frame types that are supported by the device.
     * @param[out] types
     * @return Status
     */
    virtual aditof::Status
    getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) = 0;

    /**
     * @brief Set the device frame type to the given type
     * @param details - frame details structure containing the frame type
     * @return Status
     */
    virtual aditof::Status
    setFrameType(const aditof::FrameDetails &details) = 0;

    /**
     * @brief Program the devices with the given firmware
     * @param firmware - chunk of data representin the firmware
     * @param size - the size of the firmware data in bytes
     * @return Status
     */
    virtual aditof::Status program(const uint8_t *firmware, size_t size) = 0;

    /**
     * @brief Request a frame from the device
     * @param buffer - a valid location where the new frame should be stored.
     * The size of the frame is known (cached) internally and gets updated each
     * time setFrameType() is called.
     * @return Status
     */
    virtual aditof::Status getFrame(uint16_t *buffer) = 0;

    /**
     * @brief Read the EEPROM memory of the device starting from the given
     * address.
     * @param address - the memory location to start reading from
     * @param data - a valid location to store the content read from EEPROM
     * @param length - the number of bytes to read
     * @return Status
     */
    virtual aditof::Status readEeprom(uint32_t address, uint8_t *data,
                                      size_t length) = 0;

    /**
     * @brief Write to the EEPROM memory of the device starting from the given
     * address.
     * @param address - the memory location to start writing to
     * @param data - the location of the content to be written to EEPROM
     * @param length - the number of bytes to write
     * @return Status
     */
    virtual aditof::Status writeEeprom(uint32_t address, const uint8_t *data,
                                       size_t length) = 0;

    /**
     * @brief Read multiple registers from AFE.
     * @param address - memory location pointing to addresses of registers to be
     * read
     * @param data - a valid location to store the content read from AFE
     * registers
     * @param length - the number of registers to read
     * @return Status
     */
    virtual aditof::Status readAfeRegisters(const uint16_t *address,
                                            uint16_t *data, size_t length) = 0;

    /**
     * @brief Write to multiple AFE registers.
     * @param address - memory location pointing to addresses of registers to be
     * written
     * @param data - the location of the content to be written to AFE registers
     * @param length - the number of registers to write
     * @return Status
     */
    virtual aditof::Status writeAfeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length) = 0;

    /**
     * @brief Read the AFE temperature
     * @param[out] temperature - the variable where the read temperature is
     * stored
     * @return Status
     */
    virtual aditof::Status readAfeTemp(float &temperature) = 0;

    /**
     * @brief Read the laser temperature
     * @param[out] temperature - the variable where the read temperature is
     * stored
     * @return Status
     */
    virtual aditof::Status readLaserTemp(float &temperature) = 0;

    /**
     * @brief Make device cache calibration parameters for a given camera mode
     * @param mode - a camera specific mode
     * @param gain - the gain correction
     * @param offset - the offset correction
     * @param range - the maximum range, in mm, for the given mode
     * @return Status
     */
    virtual aditof::Status setCalibrationParams(const std::string &mode,
                                                float gain, float offset,
                                                int range) = 0;

    /**
     * @brief Use the cached calibration parameters to apply corrections to the
     * given frame.
     * @param frame - the content of a frame to which the
     * calibration(correction) should be done
     * @param mode - the camera mode which allows the device (internally) to
     * figure out the right calibration parameters
     * @return Status
     */
    virtual aditof::Status applyCalibrationToFrame(uint16_t *frame,
                                                   const std::string &mode) = 0;
    /**
     * @brief Get a structure that contains information about the instance of
     * the device
     * @param[out] details - the variable where the device details should be
     * stored
     * @return Status
     */
    virtual aditof::Status getDetails(aditof::DeviceDetails &details) const = 0;
};

} // namespace aditof

#endif // DEVICE_INTERFACE_H
