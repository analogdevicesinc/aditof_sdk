#include <algorithm>
#include <array>
#include <memory>

class Chicony_EEPROM
{
    public:
        void Data_init(uint16_t gunCsTable[], uint16_t gunIspRegCmn[][2],uint16_t gunIspRegMode[][158][2],uint16_t gunIspRamMode[][73],uint16_t gunCtrlCmn[]);
        void ReadEEPROMVersion(uint16_t Edata[][2],uint16_t &count_v);
        void SetImageFormat(uint16_t mode,uint16_t Fdata[][2],uint16_t &count_v,uint16_t gunIspRegMode[][158][2]);
        bool ReadCalEEPROM(uint16_t Edata[][2],uint16_t gunCtrlCmn_get[][2],uint16_t &gunCtrlCmn_get_size
                                            ,uint16_t gunIspRegCmn_get[][2],uint16_t &gunIspRegCmn_get_size
                                            ,uint16_t gunCtrlMode_get[][34][2],uint16_t gunCtrlMode_get_size[]
                                            ,uint16_t gunIspRegMode_get[][245][2],uint16_t gunIspRegMode_get_size[]
                                            ,uint16_t gunIspRamMode_get[][73][2],uint16_t gunIspRamMode_get_size[]);
        bool ReadCSEEPROM(uint16_t gunCtrlCmn[],uint16_t gunCsTable_get[][2],uint16_t &gunCsTable_get_size);
        void PowerUpSequence( uint16_t Pdata[][2],uint16_t &count_v ,uint16_t gunCsTable[]);
        uint16_t TofGetDefExpValue(uint16_t unMode,uint16_t gunCtrlMode[][34]);
        void TofSetCcdDummy_data(uint16_t gunIspRamMode[][73],uint16_t unCcdDummy,uint16_t SetCcd_get[][2],uint16_t &SetCcd_get_size,uint16_t gunRangeMode);

        void TofSetExpValue_Data(uint16_t &unExp , uint16_t *unHdExp,uint16_t gunRangeMode, uint16_t gunCtrlMode[][34],
                                uint16_t gunVdInitialOffset ,uint16_t gunIspRegMode[][158][2],uint16_t gunIspRamMode[][73],
                                uint16_t &unLongNum,uint16_t &unShortNum,uint16_t &unLmsNum,uint16_t &gunIdlePeriod ,uint16_t &gunCcdDummy,int   &iReadSize2,int   &iHdExp );

        void wait(uint16_t time);
        void sensorPowerUp_Data(uint16_t unData, uint16_t SPU_D[][2],uint16_t &SPU_D_size,uint16_t gunExSync);
        void sensorPowerDown_Data(uint16_t SPD_D[][2],uint16_t &SPD_D_size,uint16_t gunExSync);
        void TofSetEmissionEnable_Data(uint16_t unEnable,uint16_t unData,uint16_t TSEE_D[][2],uint16_t &TSEE_D_size);
        void GetFlag(uint16_t &unTalFlag,uint16_t gunCtrlCmn[]);
        void GetTMC_data(uint16_t SPD_D[][2]);
        void GetLDFlag(uint16_t &unLdFlag,uint16_t gunCtrlMode[][34],uint16_t gunRangeMode);
        void SetVD(uint16_t VD_D[2],uint16_t gunCtrlMode[][34],uint16_t gunIspRamMode[][73],uint16_t unMode);
        void SetExp_data(uint16_t SE_D[][2],uint16_t gunHdExp);
        void SetLD(uint16_t LD_D[][2],uint16_t unLdFlag);
        bool SetTAL(uint16_t TAL_D[][2],uint16_t unData ,uint16_t unTalFlag);
        void GetMD(uint16_t MD_D[2],uint16_t unMode);




};
