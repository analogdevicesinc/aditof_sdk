/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Analog Devices, Inc.
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
#ifndef EEPROM_TOOL_H
#define EEPROM_TOOL_H

#include "camera_eeprom_interface.h"
#include "eeprom_map.h"
#include <aditof/camera.h>
#include <aditof/device_construction_data.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/storage_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

class EepromTool {

  public:
    EepromTool();
    ~EepromTool();

    aditof::Status setConnection(aditof::ConnectionType connectionType,
                                 std::string ip, std::string eepromName);

    aditof::Status writeFileToEeprom(char const *filename);
    aditof::Status readEepromToFile(char const *filename);
    aditof::Status listEeproms();

  private:
    //EEPROM operations
    aditof::Status writeEeprom(const std::vector<uint8_t> data);
    aditof::Status readEeprom(std::vector<uint8_t> &data);
    //File operations
    static aditof::Status readFile(char const *filename,
                                   std::vector<uint8_t> &);
    static aditof::Status writeFile(char const *filename,
                                    const std::vector<uint8_t>);

  private:
    aditof::DeviceConstructionData m_devData;
    std::shared_ptr<aditof::StorageInterface> m_eeprom;
    std::shared_ptr<CameraEepromInterface> m_camera_eeprom;
    std::shared_ptr<aditof::DepthSensorInterface> m_device;
    std::shared_ptr<aditof::StorageInterface> m_storage;
};

#endif // EEPROM_TOOL_H
