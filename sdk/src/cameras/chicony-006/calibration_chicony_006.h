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
#ifndef CALIBRATION_CHICONY_006_H
#define CALIBRATION_CHICONY_006_H

#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>
#include <aditof/storage_interface.h>
#include <iostream>
#include <list>
#include <memory>
#include <stdint.h>
#include <unordered_map>

class CalibrationChicony006 {
  public:
    CalibrationChicony006();
    ~CalibrationChicony006();

  public:
    aditof::Status
    initialize(std::shared_ptr<aditof::DepthSensorInterface> sensor,
               std::shared_ptr<aditof::StorageInterface> eeprom);
    aditof::Status close();
    aditof::Status setMode(const std::string &mode);

  private:
    aditof::Status SetEEPROMData_0(uint16_t Gdata[][2], uint16_t Gdata_size,
                                   uint16_t SetData[]);
    aditof::Status SetEEPROMData_1(uint16_t Gdata[][2], uint16_t Gdata_size,
                                   uint16_t SetData[][2]);
    aditof::Status TofEepromRead(uint16_t unAddr, uint16_t *punData,
                                 uint16_t unSlave);
    aditof::Status TofEepromWrite(uint16_t unAddr, uint16_t punData);

    aditof::Status TofSendCsTable(uint16_t *punSndTbl, uint32_t ulWords);
    aditof::Status unWriteTofRamReg(uint16_t *punReg, uint16_t unReg,
                                    uint16_t unSetNum);
    aditof::Status SetExposureDelay(uint16_t unMode, uint16_t unVdInitOfst);
    aditof::Status unReadTofRamReg(uint16_t *punRegAddr, uint16_t *punReg,
                                   uint16_t unRegNum);
    aditof::Status GetExposureDelay(uint16_t unMode, uint16_t *punVdInitOfst);
    aditof::Status GetIdlePeriod(uint16_t unMode, uint16_t *punIdlePeriod);
    aditof::Status TofChangeRangeMode(uint16_t unMode);
    aditof::Status TofSetCcdDummy(uint16_t unCcdDummy);
    aditof::Status TofSetExpValue(uint16_t unExp, uint16_t *unHdExp);
    aditof::Status TofSetEmissionEnable(uint16_t unEnable);
    aditof::Status sensorPowerUp();
    aditof::Status sensorPowerDown();

  private:
    std::shared_ptr<aditof::DepthSensorInterface> m_sensor;
    std::shared_ptr<aditof::StorageInterface> m_eeprom;
};

#endif /*CALIBRATION_CHICONY_006_H*/
