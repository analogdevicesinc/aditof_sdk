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

#include <aditof/device_interface.h>
#include <aditof/status_definitions.h>
#include <iostream>
#include <list>
#include <memory>
#include <stdint.h>
#include <unordered_map>

typedef struct _sTOF_PULSE_PARAM {
    uint16_t unLdWidth;
    uint16_t unSubWidth;
    uint16_t unA0Phase;
    uint16_t unA1Phase;
    uint16_t unA2Phase;
    uint16_t unSubPhase;
}sTOF_PULSE_PARAM;

class CalibrationChicony006 {
  public:
    CalibrationChicony006();
    ~CalibrationChicony006();

  public:
    aditof::Status initialize(std::shared_ptr<aditof::DeviceInterface> device);
    aditof::Status setMode(uint16_t mode);

  private:
      aditof::Status AfeRegWrite(uint16_t unAddr, uint16_t unData);
      aditof::Status AfeRegRead(uint16_t* punRcvBuf, uint32_t ulWords, uint16_t unRegAdr);
      aditof::Status unWriteTofRamReg(uint16_t *punRegAddr, uint16_t unRegVal, uint16_t unSetNum);
      aditof::Status unReadTofRamReg(uint16_t *punRegAddr, uint16_t *unRegVal, uint16_t unSetNum);
      uint16_t unGetWindowT(uint16_t unWINValue);
      
      uint16_t unCalcToFPulsePhaseAndWidth(uint16_t unReg, uint16_t unWINinfo, uint16_t *punPulseWidth, uint16_t *punPulsePhase);
      aditof::Status GetTofPulse(sTOF_PULSE_PARAM *psParam);
      aditof::Status GetExposureDelay(uint16_t unMode, uint16_t *punVdInitOfst);
      aditof::Status SetExposureDelay(uint16_t unMode, uint16_t unVdInitOfst);
      uint16_t GetDefExpValue(uint16_t unMode);
      aditof::Status SetExpValue(uint16_t unExp, uint16_t *unHdExp);
      aditof::Status SetCcdDummy(uint16_t unCcdDummy);
      
      aditof::Status EepromRead(uint16_t unAddr, uint16_t *punData);
      aditof::Status ContinuousRead_1Array(uint16_t unAddr, uint16_t unTbl[], uint16_t unStartWriteArrayNum, uint16_t unConinuousNum, uint16_t *punCheckSum);
      aditof::Status ContinuousRead_2Array(uint16_t unAddr, uint16_t unTbl[][2], uint16_t unStartWriteArrayNum, uint16_t unConinuousNum, uint16_t *punCheckSum);
      aditof::Status ContinuousRead_Reserved(uint16_t unAddr, uint16_t unConinuousNum, uint16_t *punCheckSum);
      
      aditof::Status ReadCalibData(void);
      aditof::Status ReadCommonfromEEPROM(uint16_t *punCheckSum, uint16_t unCtrlTbl[], uint16_t unIspTbl[][2], uint16_t unBase);
      aditof::Status ReadModefromEEPROM(uint16_t *punCheckSum, uint16_t unCtrlTbl[], uint16_t unIspTbl[][2], uint16_t unRamTbl[], uint16_t unBase);
      aditof::Status ReadCsIniCodefromEEPROM(uint16_t _unROMByte, uint16_t *punCheckSum);

      aditof::Status DecompressionAndSendCS(void);
      aditof::Status SendCsTable(uint16_t *punSndTbl, uint32_t ulWords);

  private:
      uint16_t gunRangeMode;
      uint16_t gunExpValue;
      uint16_t gunVdInitialOffset;
      uint16_t gunIdlePeriod;
      uint16_t gunCcdDummy;
      uint16_t gunHdExp;
      uint16_t gunExpMax;
      uint16_t gunInitialized;
      std::shared_ptr<aditof::DeviceInterface> m_device;
};

#endif /*CALIBRATION_CHICONY_006_H*/
