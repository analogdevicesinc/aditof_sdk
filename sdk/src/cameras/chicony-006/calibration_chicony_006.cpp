#include "calibration_chicony_006.h"

#include "Chicony_EEPROM.h"

#include <errno.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include <glog/logging.h>

struct ethtool_value {
    uint32_t cmd;
    uint32_t data;
};

Chicony_EEPROM CEEPROM;

uint16_t gunCsTable[6032];
uint32_t gunCsTable_Size;

uint16_t gunExpValue = 0;
uint16_t gunRangeMode = 0;
unsigned short gunCcdDummy = 0;
uint16_t gunHdExp = 0; /*!< for Detect Hounds-tooth(Checker) */

/* External synchronization valid or invalid */
uint16_t gunExSync = 0; //!< 1 is enable, 0 is disable.

/* VD initial offset (Exposure delay) */
uint16_t gunVdInitialOffset = 0;
/* Idle Period (between Exposure and readout )*/
uint16_t gunIdlePeriod = 0;

/* common registers */
uint16_t gunIspRegCmn[245][2] = {0};
//ISP_REG_MODE_MAX = 158;
/* registers for each range mode */
uint16_t gunIspRegMode[2][158][2] = {0};

uint32_t gunIspRegCmn_Size = 245;   //CYFX_ARRY_SIZE(gunIspRegCmn);
uint32_t gunIspRegMode0_Size = 158; //CYFX_ARRY_SIZE(gunIspRegMode[0]);
uint32_t gunIspRegMode1_Size = 158; //CYFX_ARRY_SIZE(gunIspRegMode[1]);

/* registers for each range mode (ISATG ram register) */
uint16_t gunIspRamMode[2][73] = {0};

/* array for read EEPROM (there is not AFE registers ) */
uint16_t gunCtrlCmn[167] = {0};
uint16_t gunCtrlMode[2][34] = {0};

uint16_t PowerUp_data[2100][2] = {0}, PowerUp_data_size = 0;
uint16_t Format_data[12][2] = {0}, Format_size = 0;
uint16_t EEPROM_V_data[10][2] = {0}, EEPROM_V_size = 0;

uint16_t gunCtrlCmn_Get[167][2] = {0}, gunCtrlCmn_Get_size = 0;
uint16_t gunIspRegCmn_Get[245][2] = {0}, gunIspRegCmn_Get_size = 0;
uint16_t gunCtrlMode_Get[2][34][2] = {0}, gunCtrlMode_Get_size[2] = {0};
uint16_t gunIspRegMode_Get[2][245][2] = {0}, gunIspRegMode_Get_size[2] = {0};
uint16_t gunIspRamMode_Get[2][73][2] = {0}, gunIspRamMode_Get_size[2] = {0};
uint16_t gunCsTable_Get[6032][2] = {0}, gunCsTable_Get_size = 0;
tm *local;

CalibrationChicony006::CalibrationChicony006() {}

CalibrationChicony006::~CalibrationChicony006() {}

aditof::Status CalibrationChicony006::SetEEPROMData_0(uint16_t Gdata[][2],
                                                      uint16_t Gdata_size,
                                                      uint16_t SetData[]) {
    using namespace aditof;
    Status status = Status::OK;
    for (int i = 0; i < Gdata_size; ++i) {
        uint16_t unData, uaddr;
        uaddr = (uint16_t)(Gdata[i][0]);
        TofEepromRead(uaddr, &unData, 1);

        SetData[(Gdata[i][1])] = unData;
    }
    return status;
}

aditof::Status CalibrationChicony006::SetEEPROMData_1(uint16_t Gdata[][2],
                                                      uint16_t Gdata_size,
                                                      uint16_t SetData[][2]) {
    using namespace aditof;
    Status status = Status::OK;
    for (int i = 0; i < Gdata_size; ++i) {
        uint16_t unData, uaddr;
        uaddr = (uint16_t)(Gdata[i][0]);
        TofEepromRead(uaddr, &unData, 1);

        SetData[(Gdata[i][1])][1] = unData;
    }
    return status;
}

aditof::Status CalibrationChicony006::initialize(
    std::shared_ptr<aditof::DepthSensorInterface> sensor,

    std::shared_ptr<aditof::StorageInterface> eeprom) {
    using namespace aditof;

    aditof::Status status = Status::OK;

    m_sensor = sensor;
    m_eeprom = eeprom;

    CEEPROM.ReadEEPROMVersion(EEPROM_V_data, EEPROM_V_size);

    CEEPROM.Data_init(gunCsTable, gunIspRegCmn, gunIspRegMode, gunIspRamMode,
                      gunCtrlCmn);

    for (int i = 0; i < EEPROM_V_size; ++i)
        TofEepromRead(EEPROM_V_data[i][0], &EEPROM_V_data[i][1], 1);

    bool bread = false;
    bread = CEEPROM.ReadCalEEPROM(
        EEPROM_V_data, gunCtrlCmn_Get, gunCtrlCmn_Get_size, gunIspRegCmn_Get,
        gunIspRegCmn_Get_size, gunCtrlMode_Get, gunCtrlMode_Get_size,
        gunIspRegMode_Get, gunIspRegMode_Get_size, gunIspRamMode_Get,
        gunIspRamMode_Get_size);

    if (!bread)
        return Status::GENERIC_ERROR;

    SetEEPROMData_0(gunCtrlCmn_Get, gunCtrlCmn_Get_size, gunCtrlCmn);
    SetEEPROMData_1(gunIspRegCmn_Get, gunIspRegCmn_Get_size, gunIspRegCmn);

    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < gunCtrlMode_Get_size[j]; ++i) {
            uint16_t unData, uaddr;
            uaddr = (uint16_t)(gunCtrlMode_Get[j][i][0]);
            TofEepromRead(uaddr, &unData, 1);

            gunCtrlMode[j][(gunCtrlMode_Get[j][i][1])] = unData;
        }

        for (int i = 0; i < gunIspRegMode_Get_size[j]; ++i) {
            uint16_t unData, uaddr;
            uaddr = (uint16_t)(gunIspRegMode_Get[j][i][0]);
            TofEepromRead(uaddr, &unData, 1);

            gunIspRegMode[j][(gunIspRegMode_Get[j][i][1])][1] = unData;
        }

        for (int i = 0; i < gunIspRamMode_Get_size[j]; ++i) {
            uint16_t unData, uaddr;
            uaddr = (uint16_t)(gunIspRamMode_Get[j][i][0]);
            TofEepromRead(uaddr, &unData, 1);

            gunIspRamMode[j][(gunIspRamMode_Get[j][i][1])] = unData;
        }
    }

    bread =
        CEEPROM.ReadCSEEPROM(gunCtrlCmn, gunCsTable_Get, gunCsTable_Get_size);
    if (!bread)
        return Status::GENERIC_ERROR;

    SetEEPROMData_0(gunCsTable_Get, gunCsTable_Get_size, gunCsTable);

    if (status != Status::OK) {
        LOG(WARNING) << "Failed to Read Cal EEPROM";
        return status;
    }

    CEEPROM.PowerUpSequence(PowerUp_data, PowerUp_data_size, gunCsTable);
    for (int i = 0; i < PowerUp_data_size; ++i) {
        m_sensor->writeAfeRegisters(&PowerUp_data[i][0], &PowerUp_data[i][1],
                                    1);
    }
    LOG(INFO) << "**************power up sequence finish";

    /* TOF ISP registers */
    LOG(INFO) << "*******************TOF ISP registers ";
    TofSendCsTable((uint16_t *)gunIspRegCmn,
                   gunIspRegCmn_Size); /* for all mode */
    TofChangeRangeMode(gunRangeMode);  /* for current mode */
    LOG(INFO) << "*******************TofChangeRangeMode";

    CEEPROM.SetImageFormat(gunRangeMode, Format_data, Format_size,
                           gunIspRegMode);
    // SetImageFormat(gunRangeMode,Format_data,Format_size);
    for (int i = 0; i < Format_size; ++i) {
        m_sensor->writeAfeRegisters(&Format_data[i][0], &Format_data[i][1], 1);
    }
    LOG(INFO) << "********************Set Format finish";

    return status;
}

aditof::Status CalibrationChicony006::close() {
    using namespace aditof;

    TofSetEmissionEnable(0); //LD Power down
    sensorPowerDown();

    return Status::OK;
}

//! setMode - Sets the mode to be used for depth calibration
/*!
setMode - Sets the mode to be used for depth calibration
\param mode - Camera depth mode
*/
aditof::Status CalibrationChicony006::setMode(const std::string &mode) {
    using namespace aditof;

    aditof::Status status = Status::OK;

    if (mode == "near") {
        gunRangeMode = 0;
        TofSetEmissionEnable(0); //LD Power down
        sensorPowerDown();

        TofChangeRangeMode(0x0000);
        LOG(INFO) << "Chosen mode: " << mode.c_str();
    } else if (mode == "medium") {
        gunRangeMode = 1;
        TofSetEmissionEnable(0); //LD Power down
        sensorPowerDown();

        TofChangeRangeMode(0x0001);
        LOG(INFO) << "Chosen mode: " << mode.c_str();
    }

    TofSetEmissionEnable(1); //LD Power On
    sensorPowerUp();

    return status;
}

aditof::Status CalibrationChicony006::TofEepromRead(uint16_t unAddr,
                                                    uint16_t *punData,
                                                    uint16_t unSlave) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t ADDR_v = uint32_t(unAddr);
    uint8_t ucData[2];
    m_eeprom->read(ADDR_v, ucData, 2);
    *punData = ((uint16_t)ucData[0] << 8) | (uint16_t)ucData[1];

    return status;
}

aditof::Status CalibrationChicony006::TofEepromWrite(uint16_t unAddr,
                                                     uint16_t punData) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t ADDR_v = uint32_t(unAddr);
    uint8_t ucData[2];
    ucData[0] = (punData >> 8);
    ucData[1] = (punData & 0x00FF);
    m_eeprom->write(ADDR_v, ucData, 2);

    return status;
}

aditof::Status CalibrationChicony006::sensorPowerUp() {
    using namespace aditof;
    Status status = Status::OK;
    uint16_t addr_v, unData;

    CEEPROM.wait(20);

    addr_v = 0xC300;
    m_sensor->readAfeRegisters(&addr_v, &unData, 1);

    uint16_t SPU_D[30][2] = {0}, SPU_D_size = 0;
    CEEPROM.sensorPowerUp_Data(unData, SPU_D, SPU_D_size, gunExSync);

    for (int i = 0; i < SPU_D_size; ++i) {
        m_sensor->writeAfeRegisters(&SPU_D[i][0], &SPU_D[i][1], 1);
    }

    LOG(WARNING) << " Powered On";
    return status;
}

aditof::Status CalibrationChicony006::sensorPowerDown() {
    using namespace aditof;
    Status status = Status::OK;

    uint16_t SPD_D[30][2] = {0}, SPD_D_size = 0;
    CEEPROM.sensorPowerDown_Data(SPD_D, SPD_D_size, gunExSync);

    for (int i = 0; i < SPD_D_size; ++i) {
        m_sensor->writeAfeRegisters(&SPD_D[i][0], &SPD_D[i][1], 1);
    }

    LOG(WARNING) << " Powered Down";

    return status;
}

aditof::Status CalibrationChicony006::TofSetEmissionEnable(uint16_t unEnable) {
    using namespace aditof;
    Status status = Status::OK;
    uint16_t addr_v, unData;

    addr_v = 0xc08e;
    m_sensor->readAfeRegisters(&addr_v, &unData, 1);

    uint16_t TSEE_D[30][2] = {0}, TSEE_D_size = 0;
    CEEPROM.TofSetEmissionEnable_Data(unEnable, unData, TSEE_D, TSEE_D_size);

    for (int i = 0; i < TSEE_D_size; ++i) {
        m_sensor->writeAfeRegisters(&TSEE_D[i][0], &TSEE_D[i][1], 1);
        CEEPROM.wait(30);
    }

    if (unEnable == 1)
        LOG(WARNING) << "LD Power On";
    else
        LOG(WARNING) << "LD Power Off";

    return status;
}

aditof::Status CalibrationChicony006::TofSendCsTable(uint16_t *punSndTbl,
                                                     uint32_t ulWords) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t ulLoop = 0;
    uint16_t *punSndCurr;

    punSndCurr = punSndTbl;

    /* Continuously write CS registers */
    for (ulLoop = 0; ulLoop < ulWords; ulLoop++) {
        uint16_t addr_v = *punSndCurr;
        uint16_t data_v = *(punSndCurr + 1);
        m_sensor->writeAfeRegisters(&addr_v, &data_v, 1);
        punSndCurr += 2;
    }

    return status;
}

aditof::Status CalibrationChicony006::unWriteTofRamReg(uint16_t *punReg,
                                                       uint16_t unReg,
                                                       uint16_t unSetNum) {
    using namespace aditof;
    Status status = Status::OK;
    int i;

    for (i = 0; i < unSetNum; i++) {
        m_sensor->writeAfeRegisters(&punReg[i], &unReg, 1);
    }

    return status;
}

aditof::Status CalibrationChicony006::SetExposureDelay(uint16_t unMode,
                                                       uint16_t unVdInitOfst) {
    using namespace aditof;
    Status status = Status::OK;
    uint16_t unVdIniOfstMinus1, unVdIniOfstAdrNum;

    unVdIniOfstMinus1 = unVdInitOfst - 1;
    if (unVdIniOfstMinus1 < 1) {
        unVdIniOfstMinus1 = 1;
    }

    // if the number of address is zero, Offset is zero.
    unVdIniOfstAdrNum = gunIspRamMode[unMode][29];

    if (unVdIniOfstAdrNum > 4) {
        unVdIniOfstAdrNum = 4;
    }
    unWriteTofRamReg(&gunIspRamMode[unMode][30], unVdIniOfstMinus1,
                     unVdIniOfstAdrNum);

    return status;
}

aditof::Status CalibrationChicony006::unReadTofRamReg(uint16_t *punRegAddr,
                                                      uint16_t *punReg,
                                                      uint16_t unRegNum) {
    using namespace aditof;
    Status status = Status::OK;
    uint16_t unData0, unDataX;
    uint16_t uni;

    uint16_t addr_v; //,data_v;

    if (unRegNum > 0) {
        addr_v = punRegAddr[0];
        m_sensor->readAfeRegisters(&addr_v, &unData0, 1);
        for (uni = 1; uni < unRegNum; uni++) {
            addr_v = punRegAddr[uni];
            m_sensor->readAfeRegisters(&addr_v, &unDataX, 1);
            if (unDataX != unData0)
                printf("**************unReadTofRamReg error\n");
        }
        *punReg = unData0;
    } else {
        *punReg = 0;
    }

    return status;
}

aditof::Status
CalibrationChicony006::GetExposureDelay(uint16_t unMode,
                                        uint16_t *punVdInitOfst) {
    using namespace aditof;
    Status status = Status::OK;
    uint16_t unVdIniOfstMinus1 = 0, unVdIniOfstAdrNum;

    unVdIniOfstAdrNum = gunIspRamMode[unMode][29];
    if (unVdIniOfstAdrNum == 0) {
        *punVdInitOfst = 0;
    } else {
        unReadTofRamReg(&gunIspRamMode[unMode][30], &unVdIniOfstMinus1,
                        unVdIniOfstAdrNum);
        *punVdInitOfst = unVdIniOfstMinus1 + 1;
    }
    return status;
}

aditof::Status CalibrationChicony006::GetIdlePeriod(uint16_t unMode,
                                                    uint16_t *punIdlePeriod) {
    using namespace aditof;
    Status status = Status::OK;
    uint16_t unIdlePeriodMinus2 = 0, unIdlePeriodAdrNum;

    unIdlePeriodAdrNum = gunIspRamMode[unMode][68];
    if (unIdlePeriodAdrNum == 0) {
        *punIdlePeriod = 0;
    } else {
        unReadTofRamReg(&gunIspRamMode[unMode][69], &unIdlePeriodMinus2,
                        unIdlePeriodAdrNum);
        *punIdlePeriod = unIdlePeriodMinus2 + 2;
    }
    return status;
}

aditof::Status CalibrationChicony006::TofSetCcdDummy(uint16_t unCcdDummy) {
    using namespace aditof;
    Status status = Status::OK;

    uint16_t CCD_data[50][2] = {0}, CCD_data_size = 0;
    CEEPROM.TofSetCcdDummy_data(gunIspRamMode, unCcdDummy, CCD_data,
                                CCD_data_size, gunRangeMode);
    for (int i = 0; i < CCD_data_size; ++i) {
        m_sensor->writeAfeRegisters(
            &gunIspRamMode[gunRangeMode][CCD_data[i][0]], &CCD_data[i][1], 1);
    }

    return status;
}

aditof::Status CalibrationChicony006::TofSetExpValue(uint16_t unExp,
                                                     uint16_t *unHdExp) {
    using namespace aditof;
    Status status = Status::OK;

    uint16_t unMode;
    if (gunRangeMode == 0) {
        unMode = 0;
    } else if (gunRangeMode == 1) {
        unMode = 1;
    } else {
        unMode = 0;
    }

    gunExpValue = unExp;
    uint16_t unLongNum, unShortNum, unLmsNum;
    int iReadSize2;
    int iHdExp;
    CEEPROM.TofSetExpValue_Data(unExp, unHdExp, gunRangeMode, gunCtrlMode,
                                gunVdInitialOffset, gunIspRegMode,
                                gunIspRamMode, unLongNum, unShortNum, unLmsNum,
                                gunIdlePeriod, gunCcdDummy, iReadSize2, iHdExp);

    unWriteTofRamReg(&gunIspRamMode[unMode][1], unExp, unLongNum);
    unWriteTofRamReg(&gunIspRamMode[unMode][16], unExp / 4, unShortNum);
    unWriteTofRamReg(&gunIspRamMode[unMode][20], unExp - unExp / 4 - 1,
                     unLmsNum);

    uint16_t addr_v = 0xC3CC;
    uint16_t data_v = (unsigned short)iReadSize2;
    m_sensor->writeAfeRegisters(&addr_v, &data_v, 1);
    (*unHdExp) = iHdExp + 750;

    return status;
}

aditof::Status CalibrationChicony006::TofChangeRangeMode(uint16_t unMode) {
    using namespace aditof;
    Status status = Status::OK;

    uint16_t unData;
    uint16_t unTalFlag, unLdFlag;

    /////////// set target Mode
    gunRangeMode = unMode;

    CEEPROM.GetFlag(unTalFlag, gunCtrlCmn); /////////////// Set TAL Flag

    // TAL mode check
    if (unTalFlag != 0x0000) {
        uint16_t TMC_Data[2][2] = {0};
        CEEPROM.GetTMC_data(TMC_Data);
        for (int i = 0; i < 2; ++i) {
            m_sensor->writeAfeRegisters(&TMC_Data[i][0], &TMC_Data[i][1], 1);
        }
    }

    CEEPROM.GetLDFlag(unLdFlag, gunCtrlMode,
                      gunRangeMode); //////////// Get LD Flag

    //////////// set ISP Registers
    switch (unMode) {
    case 1:
        TofSendCsTable((uint16_t *)gunIspRegMode[unMode], gunIspRegMode1_Size);
        break;
    case 0:
    default:
        TofSendCsTable((uint16_t *)gunIspRegMode[unMode], gunIspRegMode0_Size);
        break;
    }

    //////////// set VD Duration
    uint16_t vd_data[2] = {0};
    CEEPROM.SetVD(vd_data, gunCtrlMode, gunIspRamMode, unMode);
    m_sensor->writeAfeRegisters(&vd_data[0], &vd_data[1], 1);

    /////////// set VD initial offset
    gunVdInitialOffset = gunCtrlMode[unMode][19];
    SetExposureDelay(unMode, gunVdInitialOffset);
    GetExposureDelay(unMode, &gunVdInitialOffset);

    //printf("GetExposureDelay ************\n");

    /////////// get IDLE period
    GetIdlePeriod(unMode, &gunIdlePeriod);
    //printf("GetIdlePeriod ************\n");

    ////////////////// set exposure
    //LOG(WARNING) << "set exposure";
    gunExpValue = CEEPROM.TofGetDefExpValue(
        unMode, gunCtrlMode); //  TofGetDefExpValue(unMode);
    TofSetExpValue(gunExpValue, &gunHdExp);

    uint16_t SE_D[3][2] = {0};
    CEEPROM.SetExp_data(SE_D, gunHdExp);
    for (int i = 0; i < 3; ++i)
        m_sensor->writeAfeRegisters(&SE_D[i][0], &SE_D[i][1], 1);

    TofSetCcdDummy(gunCcdDummy);

    ////////////////// set mode
    uint16_t MD_D[2] = {0};
    CEEPROM.GetMD(MD_D, unMode);
    m_sensor->writeAfeRegisters(&MD_D[0], &MD_D[1], 1);

    //set LD
    uint16_t LD_D[2][2] = {0};
    CEEPROM.SetLD(LD_D, unLdFlag);
    for (int i = 0; i < 2; ++i)
        m_sensor->writeAfeRegisters(&LD_D[i][0], &LD_D[i][1], 1);

    //TAL mode check
    unData = LD_D[0][1];
    bool btal = false;
    uint16_t TAL_D[2][2] = {0};
    btal = CEEPROM.SetTAL(TAL_D, unData, unTalFlag);

    if (btal) {
        for (int i = 0; i < 2; ++i)
            m_sensor->writeAfeRegisters(&TAL_D[i][0], &TAL_D[i][1], 1);
    }

    LOG(WARNING) << "ChangeMode:" << unMode;
    return status;
}
