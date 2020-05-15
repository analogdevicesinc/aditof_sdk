#include "calibration_chicony_006.h"
#include "tof_isp_table.h"
#include "tof_cs_table.h"

#include <glog/logging.h>

/******************* parameter *******************/
#define ROM_MAP_VERSION              (0x0082)
#define _DEBUG_PRINT_ON 1
#define _DEBUG_SETTING_OFF 1

CalibrationChicony006::CalibrationChicony006()
    : gunRangeMode(0) {
}

CalibrationChicony006::~CalibrationChicony006() {
}

aditof::Status CalibrationChicony006::unWriteTofRamReg(uint16_t *punRegAddr, uint16_t unRegVal, uint16_t unSetNum)
{
    using namespace aditof;

    aditof::Status	apiRetStatus = Status::OK;

	for(int i = 0; i < unSetNum; i++){
		AfeRegWrite(punRegAddr[i], unRegVal);
	}

	return apiRetStatus;
}

aditof::Status CalibrationChicony006::unReadTofRamReg(uint16_t *punRegAddr, uint16_t *unRegVal, uint16_t unSetNum)
{
    using namespace aditof;

    aditof::Status	apiRetStatus = Status::OK;

    for (int i = 0; i < unSetNum; i++) {
        AfeRegRead(unRegVal, 1, punRegAddr[i]);
    }

    return apiRetStatus;
}

uint16_t CalibrationChicony006::unGetWindowT(uint16_t unWINValue)
{
    uint16_t unRet;
    short unStart;
    short unEnd;

    if (unWINValue == 0x0000) {
        unRet = 0x0000;
    }
    else {
        unStart = (unWINValue >> 8);
        unEnd = (unWINValue & 0x00FF);

        if (unStart > unEnd) {
            unRet = 0xFFFF;
        }
        else {
            unRet = unEnd - unStart + 1;
        }
    }

    return unRet;
}

uint16_t CalibrationChicony006::unCalcToFPulsePhaseAndWidth(uint16_t unReg, uint16_t unWINinfo, uint16_t *punPulseWidth, uint16_t *punPulsePhase )
{
	uint16_t unRet;
	uint16_t unPosEdge,unNegEdge;
	uint16_t unPulseWidth;
	uint16_t unMargin;
	uint16_t unWindowT;

	//check unWininfo
	unWindowT = unGetWindowT(unWINinfo);
	if( unWindowT == 0x0000 ){
		*punPulsePhase = 0;

		unRet = 0x0000;
		return unRet;
	}
	if( unWindowT == 0xFFFF ){
		unRet = 0xFFFF;
		return unRet;
	}

	/* get positive edge and negative edge */
	unPosEdge = unReg & 0x00FF;
	unNegEdge = (unReg >> 8);

	/* set phase */
	*punPulsePhase = unPosEdge;

	/* calculate width*/
	if( unWindowT == 1 ){
		if(unPosEdge >= unNegEdge){
			unRet = 0xFFFF;
			return unRet;
		}
		unPulseWidth = unNegEdge - unPosEdge;
		unRet = 0x0000;

	}else{
		if( unWindowT > 2 ){
			unMargin = (unWindowT - 2)*128;
		}else{
			unMargin = 0;
		}
		unPulseWidth = (0x80 - unPosEdge) + unMargin + unNegEdge;
		unRet = 0x0000;
	}
	*punPulseWidth = unPulseWidth;

	return unRet;
}

aditof::Status CalibrationChicony006::GetTofPulse(sTOF_PULSE_PARAM *psParam)
{
    using namespace aditof;

    aditof::Status	apiRetStatus = Status::OK;
	uint16_t unA0=0, unA1=0, unA2=0, unSub=0;
	uint16_t unA0num, unA1num, unA2num, unSubnum;
	uint16_t unRet[4];
	uint16_t unMode;

	/* get register values */
	if (gunRangeMode == 0) {
		unMode = 0;
	} else if (gunRangeMode == 1) {
		unMode = 1;
	}else {
		unMode = 0;
	}
	//[15:12]a number of SUBpulse address
	//[11:8]a number of A0pulse address
	//[7:4]a number of A1pulse address
	//[3:0]a number of A2pulse address
	unA2num = (gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_ADR_NUM] & 0x000F );
	unA1num = ((gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_ADR_NUM] & 0x00F0)>> 4);
	unA0num = ((gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_ADR_NUM] & 0x0F00)>> 8);
	unSubnum = (gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_ADR_NUM] >> 12 );

	// read pulse value from registers
	unReadTofRamReg(&(gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_A0_1]), &unA0, unA0num);
	unReadTofRamReg(&(gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_A1_1]), &unA1, unA1num);
	unReadTofRamReg(&(gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_A2_1]), &unA2, unA2num);
	unReadTofRamReg(&(gunIspRamMode[unMode][ISP_RAM_MODE_PHASE_SUB_1]), &unSub, unSubnum);

	/* calculate values */
	unRet[0] = unCalcToFPulsePhaseAndWidth(unA0, gunCtrlMode[unMode][CTRL_MODE_LD_A0_WIN],&(psParam->unLdWidth), &(psParam->unA0Phase));
	unRet[1] = unCalcToFPulsePhaseAndWidth(unA1, gunCtrlMode[unMode][CTRL_MODE_LD_A1_WIN],&(psParam->unLdWidth), &(psParam->unA1Phase));
	unRet[2] = unCalcToFPulsePhaseAndWidth(unA2, gunCtrlMode[unMode][CTRL_MODE_LD_A2_WIN],&(psParam->unLdWidth), &(psParam->unA2Phase));
	unRet[3] = unCalcToFPulsePhaseAndWidth(unSub, gunCtrlMode[unMode][CTRL_MODE_SUB_WIN],&(psParam->unSubWidth), &(psParam->unSubPhase));

	if( (unRet[0] == 0xFFFF) || (unRet[1] == 0xFFFF) || (unRet[2] == 0xFFFF) || (unRet[3] == 0xFFFF) ){
        LOG(WARNING) << "ERROR! phase registers value is failed";
		apiRetStatus = Status::GENERIC_ERROR;
	}else{
		apiRetStatus = Status::OK;
	}

	return apiRetStatus;
}

aditof::Status CalibrationChicony006::GetExposureDelay(uint16_t unMode, uint16_t *punVdInitOfst)
{
    using namespace aditof;

    uint16_t unVdIniOfstMinus1 = 0, unVdIniOfstAdrNum;

    unVdIniOfstAdrNum = gunIspRamMode[unMode][ISP_RAM_MODE_VD_INI_OFST_ADR_NUM];
    if (unVdIniOfstAdrNum == 0) {
        *punVdInitOfst = 0;
    }
    else {
        unReadTofRamReg(&gunIspRamMode[unMode][ISP_RAM_MODE_VD_INIT_OFST_ADR1], &unVdIniOfstMinus1, unVdIniOfstAdrNum);
        *punVdInitOfst = unVdIniOfstMinus1 + 1;
    }
    return Status::OK;
}

aditof::Status CalibrationChicony006::SetCcdDummy(uint16_t unCcdDummy)
{
    using namespace aditof;
    
    aditof::Status	apiRetStatus = Status::OK;
    uint16_t unMode, unAddrNum;
    uint16_t uni;

    if (gunRangeMode == 0) {
        unMode = 0;
    }
    else if (gunRangeMode == 1) {
        unMode = 1;
    }
    else {
        unMode = 0;
    }
    unAddrNum = gunIspRamMode[unMode][ISP_RAM_MODE_DMMY_TRNS_NUM];

    for (uni = ISP_RAM_MODE_DMMY_TRNS_ADR1; uni<ISP_RAM_MODE_DMMY_TRNS_ADR1 + unAddrNum; uni++) {
        AfeRegWrite(gunIspRamMode[unMode][uni], unCcdDummy);
    }
    return apiRetStatus;
}

aditof::Status CalibrationChicony006::SetExpValue(uint16_t unExp, uint16_t *unHdExp)
{
    using namespace aditof;

    aditof::Status	apiRetStatus = Status::OK;
    float fClkAn, fHdAlpha;
    int   iHdAlpha;
    int   iHdExp;
    int   iReadSize2;
    int   iVdDuration, iLdPlsDuty, iTofSeqIniOfst;
    uint16_t unMode;
    int i;
    int iPlsModeValSum;
    int iNumClkInHd;
    int iBetaNum;
    int iNumHdInReatout;
    int iVdIniOfst;
    int iTofEmtPeriodOfst;
    int iNumHdInIdlePeri;
    uint16_t unLongNum, unShortNum, unLmsNum;

    // [preparation]
    if (gunRangeMode == 0) {
        unMode = 0;
    }
    else if (gunRangeMode == 1) {
        unMode = 1;
    }
    else {
        unMode = 0;
    }
    gunExpValue = unExp;
    iVdDuration = gunCtrlMode[unMode][CTRL_MODE_VD_DURATION];
    iTofSeqIniOfst = gunCtrlMode[unMode][CTRL_MODE_TOF_SEQ_INI_OFST];
    iLdPlsDuty = gunCtrlMode[unMode][CTRL_MODE_LD_PLS_DUTY];
    iNumClkInHd = gunCtrlMode[unMode][CTRL_MODE_NUM_CLK_IN_HD];
    iBetaNum = gunCtrlMode[unMode][CTRL_MODE_BETA_NUM];
    iNumHdInReatout = gunCtrlMode[unMode][CTRL_MODE_NUM_HD_IN_READ];
    iVdIniOfst = gunVdInitialOffset;
    iTofEmtPeriodOfst = gunCtrlMode[unMode][CTRL_MODE_TOF_EMT_PERI_OFST];
    iNumHdInIdlePeri = gunIdlePeriod;

    // SUM(PLS_MODE_VAL0-9)
    iPlsModeValSum = 0;
    for (i = ISP_REG_MODE_PLS_MOD_VAL0; i <= ISP_REG_MODE_PLS_MOD_VAL9; i++) {
        iPlsModeValSum += gunIspRegMode[unMode][i][1];
    }

#if _DEBUG_PRINT_ON
    LOG(WARNING) << "iVdDuration: "     << iVdDuration;
    LOG(WARNING) << "iTofSeqIniOfst: "  << iTofSeqIniOfst;
    LOG(WARNING) << "iLdPlsDuty: "      << iLdPlsDuty;
    LOG(WARNING) << "gunIdlePeriod: "   << gunIdlePeriod;
    LOG(WARNING) << "iVdIniOfst: "      << iVdIniOfst;
#endif

    // [a] (number of clocks in a alpha)
    // fClkAn = TOF_SEQ_INI_OFST+unExp*LD_PLS_DUTY-2+ROUNDSUM(PLS_MODE_VAL0-9)*unExp/40
    fClkAn = iTofSeqIniOfst + (unExp * iLdPlsDuty) - 2 + ((float)(iPlsModeValSum*unExp) / 40.0f);

#if _DEBUG_PRINT_ON
    LOG(WARNING) << "fClkAn: " << fClkAn;
#endif
    // [b] (number of HDs in a alpha)
    // fHdAlpha =ROUNDUP([a]/NUM_CLK_IN_HD
    fHdAlpha = fClkAn / (float)iNumClkInHd;
    iHdAlpha = (int)fHdAlpha;
    if (fHdAlpha > iHdAlpha) {
        iHdAlpha += 1;
    }
#if _DEBUG_PRINT_ON
    LOG(WARNING) << "iHdAlpha: " << iHdAlpha;
#endif

    // [c] (number of HDs during an exposure period)
    // HdExp = ([b]+[b]+[b])*BETA_NUM+TOF_EMT_PERIOD_OFST
    iHdExp = (3 * iHdAlpha) * iBetaNum + iTofEmtPeriodOfst;
#if _DEBUG_PRINT_ON
    LOG(WARNING) << "iHdExp: " << iHdExp;
#endif

    // [*1] (READ_SIZE2 )
    //READ_SIZE2=VD_INI_OFST+[c]+READ_SIZE2_OFFSET7+ NUM_HD_IN_IDLE_PERIOD
    iReadSize2 = iVdIniOfst + iHdExp + 7 + iNumHdInIdlePeri;

    // [*2] (dummy transfer size : number of HDs after a read out period)
    // CCD_DUMMY_HD = VD_DURATION-VD_INI_OFST-[c]-NUM_HD_IN_READOUT - NUM_HD_IN_IDLE_PERIOD
    gunCcdDummy = iVdDuration - iVdIniOfst - iHdExp - iNumHdInReatout - iNumHdInIdlePeri;


    // [Note]
    // [1]target addresses below are decided depending on CS_INI_CODE
    //    this function should be committed during a CS read out duration
    unLongNum = (gunIspRamMode[unMode][ISP_RAM_MODE_EXP_ADR_NUM] >> 8);
    unShortNum = ((gunIspRamMode[unMode][ISP_RAM_MODE_EXP_ADR_NUM] & 0x00F0) >> 4);
    unLmsNum = (gunIspRamMode[unMode][ISP_RAM_MODE_EXP_ADR_NUM] & 0x000F);
    unWriteTofRamReg(&gunIspRamMode[unMode][ISP_RAM_MODE_EXP_ADR_LONG1], unExp, unLongNum);
    unWriteTofRamReg(&gunIspRamMode[unMode][ISP_RAM_MODE_EXP_ADR_SHORT1], unExp / 4, unShortNum);
    unWriteTofRamReg(&gunIspRamMode[unMode][ISP_RAM_MODE_EXP_ADR_LMS1], unExp - unExp / 4 - 1, unLmsNum);

    AfeRegWrite(0xC3CC, (unsigned short)iReadSize2);

    (*unHdExp) = iHdExp + 750;	//output for Hounds-tooth(Checker)

#if _DEBUG_PRINT_ON
   LOG(WARNING) << "iReadSize2: "   << iReadSize2;
   LOG(WARNING) << "gunCcdDummy: "  << gunCcdDummy;
   LOG(WARNING) << "HdExp: "        << *unHdExp;
#endif

    return apiRetStatus;
}

aditof::Status CalibrationChicony006::SetExposureDelay(uint16_t unMode, uint16_t unVdInitOfst)
{
    using namespace aditof;
    
    uint16_t unVdIniOfstMinus1,unVdIniOfstAdrNum;

	unVdIniOfstMinus1 = unVdInitOfst - 1;
	if(unVdIniOfstMinus1 < 1 ){
		unVdIniOfstMinus1 = 1;
	}

	// if the number of address is zero, Offset is zero.
	unVdIniOfstAdrNum = gunIspRamMode[unMode][ISP_RAM_MODE_VD_INI_OFST_ADR_NUM];
	if( unVdIniOfstAdrNum == 0){
		return Status::GENERIC_ERROR;
	}

	if( unVdIniOfstAdrNum > 4){
		unVdIniOfstAdrNum = 4;
	}
	unWriteTofRamReg( &gunIspRamMode[unMode][ISP_RAM_MODE_VD_INIT_OFST_ADR1]  , unVdIniOfstMinus1 , unVdIniOfstAdrNum );

	return Status::OK;
}

uint16_t CalibrationChicony006::GetDefExpValue(uint16_t unMode)
{
	uint16_t unExpDef=0,unExpMax=400,unExpAe1=0,unExpAe3=0;
	if( unMode >= 0 && unMode < 2 ){
		unExpDef = gunCtrlMode[unMode][CTRL_MODE_EXP_DFLT];
		unExpMax = gunCtrlMode[unMode][CTRL_MODE_EXP_MAX];
		unExpAe1 = gunCtrlMode[unMode][CTRL_MODE_AE_EXP1];
		unExpAe3 = gunCtrlMode[unMode][CTRL_MODE_AE_EXP3];
	}else{
		unExpDef = gunCtrlMode[0][CTRL_MODE_EXP_DFLT];
		unExpMax = gunCtrlMode[0][CTRL_MODE_EXP_MAX];
		unExpAe1 = gunCtrlMode[0][CTRL_MODE_AE_EXP1];
		unExpAe3 = gunCtrlMode[0][CTRL_MODE_AE_EXP3];
	}

	//set max exposure
	if( unExpMax == 0){
		if(unExpAe3 == 0){
			gunExpMax = 400;
		}else{
			gunExpMax = unExpAe3;
		}
	}else{
		gunExpMax = unExpMax;
	}

	//set default exposure
	if( unExpDef <= 0 || unExpDef > unExpMax){
		if(unExpAe1 == 0){
			unExpDef = 100;
		}else{
			unExpDef = unExpAe1;
		}
	}

	return unExpDef;
}

aditof::Status CalibrationChicony006::EepromRead(uint16_t unAddr, uint16_t *punData)
{
    using namespace aditof;
    
    aditof::Status apiRetStatus = Status::OK;
	uint8_t data[2];
		
	apiRetStatus = m_device->readEeprom((uint32_t)unAddr, data, 2);
	
	*punData = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
	return apiRetStatus;
}

aditof::Status CalibrationChicony006::ContinuousRead_1Array( uint16_t unAddr, uint16_t unTbl[] , uint16_t unStartWriteArrayNum, uint16_t unConinuousNum, uint16_t *punCheckSum)
{
    using namespace aditof;

    aditof::Status apiRetStatus = Status::OK;
	uint16_t unData,unLoop,unCheckSum = *punCheckSum;
	for( unLoop = 0 ; unLoop < unConinuousNum ; unLoop++ ){
		apiRetStatus = EepromRead((uint16_t)(unAddr + unLoop*2) , &unData);
		unTbl[unStartWriteArrayNum+unLoop] = unData;
		unCheckSum = unCheckSum ^ unData;
	}
	*punCheckSum = unCheckSum;
	return apiRetStatus;
}

aditof::Status CalibrationChicony006::ContinuousRead_2Array( uint16_t unAddr, uint16_t unTbl[][2] , uint16_t unStartWriteArrayNum, uint16_t unConinuousNum, uint16_t *punCheckSum )
{
    using namespace aditof;

    aditof::Status apiRetStatus = Status::OK;
	uint16_t unData,unLoop,unCheckSum = *punCheckSum;
	for( unLoop = 0 ; unLoop < unConinuousNum ; unLoop++ ){
		apiRetStatus = EepromRead((uint16_t)(unAddr + unLoop*2) , &unData);
		unTbl[unStartWriteArrayNum+unLoop][1] = unData;
		unCheckSum = unCheckSum ^ unData;
	}
	*punCheckSum = unCheckSum;
	return apiRetStatus;
}

aditof::Status CalibrationChicony006::ContinuousRead_Reserved( uint16_t unAddr, uint16_t unConinuousNum, uint16_t *punCheckSum )
{
    using namespace aditof;
    
    aditof::Status apiRetStatus = Status::OK;
	uint16_t unData,unLoop,unCheckSum = *punCheckSum;
	for( unLoop = 0 ; unLoop < unConinuousNum ; unLoop++ ){
		apiRetStatus = EepromRead((uint16_t)(unAddr + unLoop*2) , &unData);
		unCheckSum = unCheckSum ^ unData;
	}
	*punCheckSum = unCheckSum;
	return apiRetStatus;
}


aditof::Status CalibrationChicony006::ReadCommonfromEEPROM(uint16_t *punCheckSum, uint16_t unCtrlTbl[],  uint16_t unIspTbl[][2], uint16_t unBase)
{
    using namespace aditof;
	
    aditof::Status	apiRetStatus = Status::OK;
	uint16_t unCheckSum = *punCheckSum, unDummy;

	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CAM_MODEL_NAME_0     ), unCtrlTbl, CTRL_CMN_CAM_MODEL_NAME_0     , 16, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CS_MODEL_NAME_0      ), unCtrlTbl, CTRL_CMN_CS_MODEL_NAME_0      , 16, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_TOFIS_MODEL_NAME_0   ), unCtrlTbl, CTRL_CMN_TOFIS_MODEL_NAME_0   , 16, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_MODEL_NAME_0    ), unCtrlTbl, CTRL_CMN_LENS_MODEL_NAME_0    , 16, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CAM_SER_NO           ), unCtrlTbl, CTRL_CMN_CAM_SER_NO           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_MAP_VERSION          ), unCtrlTbl, CTRL_CMN_MAP_VERSION          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EX_CAM_SER_NO        ), unCtrlTbl, CTRL_CMN_EX_CAM_SER_NO        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CALIB_DATE           ), unCtrlTbl, CTRL_CMN_CALIB_DATE           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CALIB_SER_NO         ), unCtrlTbl, CTRL_CMN_CALIB_SER_NO         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CAM_TYPE_CODE1       ), unCtrlTbl, CTRL_CMN_CAM_TYPE_CODE1       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CAM_TYPE_CODE2       ), unCtrlTbl, CTRL_CMN_CAM_TYPE_CODE2       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CS_MCODE_ID          ), unCtrlTbl, CTRL_CMN_CS_MCODE_ID          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_OPT_AXIS_CNTR_H      ), unCtrlTbl, CTRL_CMN_OPT_AXIS_CNTR_H      , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_OPT_AXIS_CNTR_V      ), unCtrlTbl, CTRL_CMN_OPT_AXIS_CNTR_V      , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_OPT_AXIS_TILT_ANGL   ), unCtrlTbl, CTRL_CMN_OPT_AXIS_TILT_ANGL   , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_OPT_AXIS_PAN_ANGL    ), unCtrlTbl, CTRL_CMN_OPT_AXIS_PAN_ANGL    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_FOV_H           ), unCtrlTbl, CTRL_CMN_LENS_FOV_H           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_FOV_V           ), unCtrlTbl, CTRL_CMN_LENS_FOV_V           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PRLX_H_FRM_RGBIS     ), unCtrlTbl, CTRL_CMN_PRLX_H_FRM_RGBIS     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PRLX_V_FRM_RGBIS     ), unCtrlTbl, CTRL_CMN_PRLX_V_FRM_RGBIS     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_TOFIS_PITCH          ), unCtrlTbl, CTRL_CMN_TOFIS_PITCH          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_FOCAL_LEN            ), unCtrlTbl, CTRL_CMN_FOCAL_LEN            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VALID_RANG_MODE      ), unCtrlTbl, CTRL_CMN_VALID_RANG_MODE      , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DATA_PACK_STR        ), unCtrlTbl, CTRL_CMN_DATA_PACK_STR        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_TAL_MODE_FLAG        ), unCtrlTbl, CTRL_CMN_TAL_MODE_FLAG        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CS_INI_CODE_SIZE     ), unCtrlTbl, CTRL_CMN_CS_INI_CODE_SIZE     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CHECK_SUM            ), unCtrlTbl, CTRL_CMN_CHECK_SUM            , 1, &unDummy);	//Dummy
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x00AE), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DEPTH_PLNR_X3_0      ), unCtrlTbl, CTRL_CMN_DEPTH_PLNR_X3_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DEPTH_PLNR_X2_0      ), unCtrlTbl, CTRL_CMN_DEPTH_PLNR_X2_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DEPTH_PLNR_X1_0      ), unCtrlTbl, CTRL_CMN_DEPTH_PLNR_X1_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DEPTH_PLNR_X0_0      ), unCtrlTbl, CTRL_CMN_DEPTH_PLNR_X0_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_UNDIS_X3_0      ), unCtrlTbl, CTRL_CMN_LENS_UNDIS_X3_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_UNDIS_X2_0      ), unCtrlTbl, CTRL_CMN_LENS_UNDIS_X2_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_UNDIS_X1_0      ), unCtrlTbl, CTRL_CMN_LENS_UNDIS_X1_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_UNDIS_X0_0      ), unCtrlTbl, CTRL_CMN_LENS_UNDIS_X0_0      , 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_CX_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_CX_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_CY_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_CY_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_FX_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_FX_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_FY_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_FY_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_K1_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_K1_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_K2_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_K2_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_P1_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_P1_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_P2_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_P2_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_K3_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_K3_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_K4_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_K4_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_K5_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_K5_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LENS_OPNCV_PARAM_K6_0), unCtrlTbl, CTRL_CMN_LENS_OPNCV_PARAM_K6_0, 4, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_OFST0            ), unIspTbl, ISP_REG_CMN_SHD_OFST0            , 192, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD                  ), unIspTbl, ISP_REG_CMN_SHD                  , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_X0               ), unIspTbl, ISP_REG_CMN_SHD_X0               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_XPWR0            ), unIspTbl, ISP_REG_CMN_SHD_XPWR0            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_XPWR1            ), unIspTbl, ISP_REG_CMN_SHD_XPWR1            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_XPWR2            ), unIspTbl, ISP_REG_CMN_SHD_XPWR2            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_XPWR3            ), unIspTbl, ISP_REG_CMN_SHD_XPWR3            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_Y0               ), unIspTbl, ISP_REG_CMN_SHD_Y0               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_YPWR0            ), unIspTbl, ISP_REG_CMN_SHD_YPWR0            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_YPWR1            ), unIspTbl, ISP_REG_CMN_SHD_YPWR1            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_YPWR2            ), unIspTbl, ISP_REG_CMN_SHD_YPWR2            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x02E4), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x02E6), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x02E8), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x02EA), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x02EC), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x02EE), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL0     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL0     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL1     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL1     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL2     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL2     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL3     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL3     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL4     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL4     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL5     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL5     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL6     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL6     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL7     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL7     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL8     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL8     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL9     ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL9     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL10    ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL10    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT_PIX_TH_TBL11    ), unIspTbl, ISP_REG_CMN_DFCT_PIX_TH_TBL11    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT0                ), unIspTbl, ISP_REG_CMN_DFCT0                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DFCT1                ), unIspTbl, ISP_REG_CMN_DFCT1                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHP_LOC              ), unIspTbl, ISP_REG_CMN_SHP_LOC              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_SHD_LOC              ), unIspTbl, ISP_REG_CMN_SHD_LOC              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_OUTPUT               ), unIspTbl, ISP_REG_CMN_OUTPUT               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_OUTPUTSEL            ), unIspTbl, ISP_REG_CMN_OUTPUTSEL            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_VC                   ), unIspTbl, ISP_REG_CMN_VC                   , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0316), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0318), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x031A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x031C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x031E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_GRID3                ), unIspTbl, ISP_REG_CMN_GRID3                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR1                  ), unIspTbl, ISP_REG_CMN_IR1                  , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM0              ), unIspTbl, ISP_REG_CMN_IR_GMM0              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM1              ), unIspTbl, ISP_REG_CMN_IR_GMM1              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM2              ), unIspTbl, ISP_REG_CMN_IR_GMM2              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y0            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y0            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y1            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y1            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y2            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y2            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y3            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y3            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y4            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y4            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y5            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y5            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y6            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y6            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y7            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y7            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_GMM_Y8            ), unIspTbl, ISP_REG_CMN_IR_GMM_Y8            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_UPPRTH          ), unIspTbl, ISP_REG_CMN_CHKR_UPPRTH          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_LWRTH           ), unIspTbl, ISP_REG_CMN_CHKR_LWRTH           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_START_V         ), unIspTbl, ISP_REG_CMN_CHKR_START_V         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_START_H         ), unIspTbl, ISP_REG_CMN_CHKR_START_H         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_SIZE_H          ), unIspTbl, ISP_REG_CMN_CHKR_SIZE_H          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_UPPRERR_H       ), unIspTbl, ISP_REG_CMN_CHKR_UPPRERR_H       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_UPPRERR_V       ), unIspTbl, ISP_REG_CMN_CHKR_UPPRERR_V       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_LWRERR_H        ), unIspTbl, ISP_REG_CMN_CHKR_LWRERR_H        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_LWRERR_V        ), unIspTbl, ISP_REG_CMN_CHKR_LWRERR_V        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CHKR_DET_ENA         ), unIspTbl, ISP_REG_CMN_CHKR_DET_ENA         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0350), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0352), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0354), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0356), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0358), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x035A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x035C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x035E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0360), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0362), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0364), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0366), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0368), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x036A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x036C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x036E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0370), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0372), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0374), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0376), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0378), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x037A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x037C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x037E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0380), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0382), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0384), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0386), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0388), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x038A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x038C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x038E), 1, &unCheckSum );

	*punCheckSum = unCheckSum;
	return apiRetStatus;
}

aditof::Status CalibrationChicony006::ReadModefromEEPROM(uint16_t *punCheckSum, uint16_t unCtrlTbl[],  uint16_t unIspTbl[][2], uint16_t unRamTbl[], uint16_t unBase)
{
    using namespace aditof;

    aditof::Status	apiRetStatus = Status::OK;
	uint16_t unCheckSum = *punCheckSum;
	
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_TOF_MODE_FLAG        ), unCtrlTbl, CTRL_MODE_TOF_MODE_FLAG        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LD_FLAG              ), unCtrlTbl, CTRL_MODE_LD_FLAG              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_RANGE_NEAR_LIMIT     ), unCtrlTbl, CTRL_MODE_RANGE_NEAR_LIMIT     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_RANGE_FAR_LIMIT      ), unCtrlTbl, CTRL_MODE_RANGE_FAR_LIMIT      , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DEPTH_UNIT           ), unCtrlTbl, CTRL_MODE_DEPTH_UNIT           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x000A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_MAX              ), unCtrlTbl, CTRL_MODE_EXP_MAX              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_DFLT             ), unCtrlTbl, CTRL_MODE_EXP_DFLT             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_DIST1             ), unCtrlTbl, CTRL_MODE_AE_DIST1             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_EXP1              ), unCtrlTbl, CTRL_MODE_AE_EXP1              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_SLOPE1            ), unCtrlTbl, CTRL_MODE_AE_SLOPE1            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_OFST1             ), unCtrlTbl, CTRL_MODE_AE_OFST1             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_DIST2             ), unCtrlTbl, CTRL_MODE_AE_DIST2             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_EXP2              ), unCtrlTbl, CTRL_MODE_AE_EXP2              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_SLOPE2            ), unCtrlTbl, CTRL_MODE_AE_SLOPE2            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_OFST2             ), unCtrlTbl, CTRL_MODE_AE_OFST2             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_DIST3             ), unCtrlTbl, CTRL_MODE_AE_DIST3             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_EXP3              ), unCtrlTbl, CTRL_MODE_AE_EXP3              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_SLOPE3            ), unCtrlTbl, CTRL_MODE_AE_SLOPE3            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_AE_OFST3             ), unCtrlTbl, CTRL_MODE_AE_OFST3             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0028), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x002A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x002C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x002E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0030), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0032), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0034), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0036), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0038), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x003A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x003C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x003E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_NUM          ), unRamTbl, ISP_RAM_MODE_EXP_ADR_NUM          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG1        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG1        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG2        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG2        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG3        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG3        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG4        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG4        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG5        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG5        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG6        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG6        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG7        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG7        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG8        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG8        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG9        ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG9        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG10       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG10       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG11       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG11       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG12       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG12       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG13       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG13       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG14       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG14       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LONG15       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LONG15       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_SHORT1       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_SHORT1       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_SHORT2       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_SHORT2       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_SHORT3       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_SHORT3       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_SHORT4       ), unRamTbl, ISP_RAM_MODE_EXP_ADR_SHORT4       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LMS1         ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LMS1         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LMS2         ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LMS2         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LMS3         ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LMS3         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_EXP_ADR_LMS4         ), unRamTbl, ISP_RAM_MODE_EXP_ADR_LMS4         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DMMY_TRNS_NUM        ), unRamTbl, ISP_RAM_MODE_DMMY_TRNS_NUM        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DMMY_TRNS_ADR1       ), unRamTbl, ISP_RAM_MODE_DMMY_TRNS_ADR1       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DMMY_TRNS_ADR2       ), unRamTbl, ISP_RAM_MODE_DMMY_TRNS_ADR2       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DMMY_TRNS_ADR3       ), unRamTbl, ISP_RAM_MODE_DMMY_TRNS_ADR3       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_DMMY_TRNS_ADR4       ), unRamTbl, ISP_RAM_MODE_DMMY_TRNS_ADR4       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_INI_OFST          ), unCtrlTbl, CTRL_MODE_VD_INI_OFST          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_INI_OFST_ADR_NUM  ), unRamTbl, ISP_RAM_MODE_VD_INI_OFST_ADR_NUM  , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_INIT_OFST_ADR1    ), unRamTbl, ISP_RAM_MODE_VD_INIT_OFST_ADR1    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_INIT_OFST_ADR2    ), unRamTbl, ISP_RAM_MODE_VD_INIT_OFST_ADR2    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_INIT_OFST_ADR3    ), unRamTbl, ISP_RAM_MODE_VD_INIT_OFST_ADR3    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_INIT_OFST_ADR4    ), unRamTbl, ISP_RAM_MODE_VD_INIT_OFST_ADR4    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LD_PLS_DUTY          ), unCtrlTbl, CTRL_MODE_LD_PLS_DUTY          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_DURATION          ), unCtrlTbl, CTRL_MODE_VD_DURATION          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_VD_REG_ADR           ), unRamTbl, ISP_RAM_MODE_VD_REG_ADR           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_NUM_CLK_IN_HD        ), unCtrlTbl, CTRL_MODE_NUM_CLK_IN_HD        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_BETA_NUM             ), unCtrlTbl, CTRL_MODE_BETA_NUM             , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_NUM_HD_IN_READ       ), unCtrlTbl, CTRL_MODE_NUM_HD_IN_READ       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CLK_WIDTH_U          ), unCtrlTbl, CTRL_MODE_CLK_WIDTH_U          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_CLK_WIDTH_L          ), unCtrlTbl, CTRL_MODE_CLK_WIDTH_L          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_TOF_EMT_PERI_OFST    ), unCtrlTbl, CTRL_MODE_TOF_EMT_PERI_OFST    , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_TOF_SEQ_INI_OFST     ), unCtrlTbl, CTRL_MODE_TOF_SEQ_INI_OFST     , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x009A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x009C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x009E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LD_A0_WIN            ), unCtrlTbl, CTRL_MODE_LD_A0_WIN            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LD_A1_WIN            ), unCtrlTbl, CTRL_MODE_LD_A1_WIN            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_LD_A2_WIN            ), unCtrlTbl, CTRL_MODE_LD_A2_WIN            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_SUB_WIN              ), unCtrlTbl, CTRL_MODE_SUB_WIN              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_ADR_NUM        ), unRamTbl, ISP_RAM_MODE_PHASE_ADR_NUM        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x00AA), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x00AC), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x00AE), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_1          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_1          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_2          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_2          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_3          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_3          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_4          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_4          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_5          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_5          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_6          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_6          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_7          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_7          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_SUB_8          ), unRamTbl, ISP_RAM_MODE_PHASE_SUB_8          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_1           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_1           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_2           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_2           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_3           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_3           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_4           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_4           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_5           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_5           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_6           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_6           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_7           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_7           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A0_8           ), unRamTbl, ISP_RAM_MODE_PHASE_A0_8           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_1           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_1           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_2           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_2           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_3           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_3           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_4           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_4           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_5           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_5           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_6           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_6           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_7           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_7           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A1_8           ), unRamTbl, ISP_RAM_MODE_PHASE_A1_8           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_1           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_1           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_2           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_2           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_3           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_3           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_4           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_4           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_5           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_5           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_6           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_6           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_7           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_7           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_PHASE_A2_8           ), unRamTbl, ISP_RAM_MODE_PHASE_A2_8           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_OFST00           ), unIspTbl, ISP_REG_MODE_LNR_OFST00           , 49, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_X0               ), unIspTbl, ISP_REG_MODE_LNR_X0               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_01          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_01          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_05          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_05          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_09          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_09          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_13          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_13          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_17          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_17          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_21          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_21          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_25          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_25          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_29          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_29          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_33          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_33          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_37          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_37          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_41          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_41          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_LNR_XPWR_45          ), unIspTbl, ISP_REG_MODE_LNR_XPWR_45          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x016C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x016E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DEPTH_OFST           ), unIspTbl, ISP_REG_MODE_DEPTH_OFST           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DEPTH_EX_OFST        ), unIspTbl, ISP_REG_MODE_DEPTH_EX_OFST        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DEPTH_SLOPE          ), unIspTbl, ISP_REG_MODE_DEPTH_SLOPE          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_IDLE_PERI_NUM        ), unRamTbl, ISP_RAM_MODE_IDLE_PERI_NUM        , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_IDLE_PERI_ADR1       ), unRamTbl, ISP_RAM_MODE_IDLE_PERI_ADR1       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_IDLE_PERI_ADR2       ), unRamTbl, ISP_RAM_MODE_IDLE_PERI_ADR2       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_IDLE_PERI_ADR3       ), unRamTbl, ISP_RAM_MODE_IDLE_PERI_ADR3       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_1Array((uint16_t)(unBase+ROM_IDLE_PERI_ADR4       ), unRamTbl, ISP_RAM_MODE_IDLE_PERI_ADR4       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RATE_ADJST0          ), unIspTbl, ISP_REG_MODE_RATE_ADJST0          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RATE_ADJST1          ), unIspTbl, ISP_REG_MODE_RATE_ADJST1          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RATE_ADJST2          ), unIspTbl, ISP_REG_MODE_RATE_ADJST2          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN0               ), unIspTbl, ISP_REG_MODE_ALIGN0               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN1               ), unIspTbl, ISP_REG_MODE_ALIGN1               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN2               ), unIspTbl, ISP_REG_MODE_ALIGN2               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN3               ), unIspTbl, ISP_REG_MODE_ALIGN3               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN4               ), unIspTbl, ISP_REG_MODE_ALIGN4               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN5               ), unIspTbl, ISP_REG_MODE_ALIGN5               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN6               ), unIspTbl, ISP_REG_MODE_ALIGN6               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN7               ), unIspTbl, ISP_REG_MODE_ALIGN7               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN8               ), unIspTbl, ISP_REG_MODE_ALIGN8               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN9               ), unIspTbl, ISP_REG_MODE_ALIGN9               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ALIGN10              ), unIspTbl, ISP_REG_MODE_ALIGN10              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_READ_SIZE0           ), unIspTbl, ISP_REG_MODE_READ_SIZE0           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_READ_SIZE1           ), unIspTbl, ISP_REG_MODE_READ_SIZE1           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_READ_SIZE3           ), unIspTbl, ISP_REG_MODE_READ_SIZE3           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_READ_SIZE4           ), unIspTbl, ISP_REG_MODE_READ_SIZE4           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_READ_SIZE5           ), unIspTbl, ISP_REG_MODE_READ_SIZE5           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_READ_SIZE6           ), unIspTbl, ISP_REG_MODE_READ_SIZE6           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_READ_SIZE7           ), unIspTbl, ISP_REG_MODE_READ_SIZE7           , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI0                 ), unIspTbl, ISP_REG_MODE_ROI0                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI1                 ), unIspTbl, ISP_REG_MODE_ROI1                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI2                 ), unIspTbl, ISP_REG_MODE_ROI2                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI3                 ), unIspTbl, ISP_REG_MODE_ROI3                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI4                 ), unIspTbl, ISP_REG_MODE_ROI4                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI5                 ), unIspTbl, ISP_REG_MODE_ROI5                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI6                 ), unIspTbl, ISP_REG_MODE_ROI6                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_ROI7                 ), unIspTbl, ISP_REG_MODE_ROI7                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_LNR12_SW          ), unIspTbl, ISP_REG_MODE_IR_LNR12_SW          , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x01BC), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x01BE), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_GRID0                ), unIspTbl, ISP_REG_MODE_GRID0                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_GRID1                ), unIspTbl, ISP_REG_MODE_GRID1                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_GRID2                ), unIspTbl, ISP_REG_MODE_GRID2                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_XPWR_1         ), unIspTbl, ISP_REG_MODE_RAWNR_XPWR_1         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_XPWR_5         ), unIspTbl, ISP_REG_MODE_RAWNR_XPWR_5         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_XPWR_9         ), unIspTbl, ISP_REG_MODE_RAWNR_XPWR_9         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL00       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL00       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL01       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL01       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL02       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL02       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL03       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL03       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL04       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL04       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL05       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL05       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL06       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL06       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL07       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL07       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL08       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL08       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL09       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL09       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL10       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL10       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL11       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL11       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BL_TBL12       ), unIspTbl, ISP_REG_MODE_RAWNR_BL_TBL12       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_MED            ), unIspTbl, ISP_REG_MODE_RAWNR_MED            , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_SAT_TH         ), unIspTbl, ISP_REG_MODE_RAWNR_SAT_TH         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL00       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL00       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL01       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL01       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL02       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL02       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL03       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL03       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL04       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL04       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL05       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL05       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL06       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL06       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL07       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL07       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL08       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL08       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL09       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL09       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL10       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL10       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL11       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL11       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_RAWNR_BK_TBL12       ), unIspTbl, ISP_REG_MODE_RAWNR_BK_TBL12       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_COR0                 ), unIspTbl, ISP_REG_MODE_COR0                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_COR1                 ), unIspTbl, ISP_REG_MODE_COR1                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_COR2                 ), unIspTbl, ISP_REG_MODE_COR2                 , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CORB0                ), unIspTbl, ISP_REG_MODE_CORB0                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CORB1                ), unIspTbl, ISP_REG_MODE_CORB1                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CORB2                ), unIspTbl, ISP_REG_MODE_CORB2                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CORF0                ), unIspTbl, ISP_REG_MODE_CORF0                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CORF1                ), unIspTbl, ISP_REG_MODE_CORF1                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CORF2                ), unIspTbl, ISP_REG_MODE_CORF2                , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_IR_ENABLED_COR       ), unIspTbl, ISP_REG_MODE_IR_ENABLED_COR       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DEPTH1               ), unIspTbl, ISP_REG_MODE_DEPTH1               , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_CONTROL              ), unIspTbl, ISP_REG_MODE_CONTROL              , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_CTRL         ), unIspTbl, ISP_REG_MODE_PLS_MOD_CTRL         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL0         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL0         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL1         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL1         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL2         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL2         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL3         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL3         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL4         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL4         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL5         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL5         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL6         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL6         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL7         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL7         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL8         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL8         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_PLS_MOD_VAL9         ), unIspTbl, ISP_REG_MODE_PLS_MOD_VAL9         , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DEPTH_ALF_GAIN       ), unIspTbl, ISP_REG_MODE_DEPTH_ALF_GAIN       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DEPTH_ALF_OFST       ), unIspTbl, ISP_REG_MODE_DEPTH_ALF_OFST       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DPTHDET_START_H      ), unIspTbl, ISP_REG_MODE_DPTHDET_START_H      , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DPTHDET_START_V      ), unIspTbl, ISP_REG_MODE_DPTHDET_START_V      , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DPTHDET_SIZE_H       ), unIspTbl, ISP_REG_MODE_DPTHDET_SIZE_H       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_2Array((uint16_t)(unBase+ROM_DPTHDET_SIZE_V       ), unIspTbl, ISP_REG_MODE_DPTHDET_SIZE_V       , 1, &unCheckSum);
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x023E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0240), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0242), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0244), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0246), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0248), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x024A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x024C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x024E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0250), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0252), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0254), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0256), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0258), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x025A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x025C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x025E), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0260), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0262), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0264), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0266), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x0268), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x026A), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x026C), 1, &unCheckSum );
	apiRetStatus = ContinuousRead_Reserved((uint16_t)(unBase+0x026E), 1, &unCheckSum );

	*punCheckSum = unCheckSum;
	return apiRetStatus;
}

aditof::Status CalibrationChicony006::ReadCsIniCodefromEEPROM( uint16_t _unROMByte , uint16_t *punCheckSum)
{
    using namespace aditof;
    
    int iTblCnt;
	int iROMCnt;
	uint16_t unEEPROM_Addr, unData, unCheckSum = *punCheckSum;
	aditof::Status apiRetStatus = Status::OK;
	

	if( _unROMByte > MAX_CS_INI_CODE_SIZE){
		return Status::GENERIC_ERROR;
	}

	iTblCnt = 0;
	for(iROMCnt = 0 ;iROMCnt < _unROMByte ; iROMCnt += 2) {
		unEEPROM_Addr = ROM_CS_INI_CODE_0 + iROMCnt;

		//size_check
		if( unEEPROM_Addr > ROM_CS_INI_CODE_END ){
			return Status::GENERIC_ERROR;
		}

		apiRetStatus = EepromRead(unEEPROM_Addr, &unData);
		gunCsTable[iTblCnt] = unData;
		unCheckSum = unCheckSum ^ unData;
		iTblCnt++;
	}
	gunCsTable_Size = iTblCnt;
	*punCheckSum = unCheckSum;

	return apiRetStatus;
}

aditof::Status CalibrationChicony006::AfeRegWrite(uint16_t unAddr, uint16_t unData)
{
    using namespace aditof;

    aditof::Status apiRetStatus = Status::OK;

	apiRetStatus = m_device->writeAfeRegisters((const uint16_t*)&unAddr,
									 (const uint16_t*)&unData, 1);

	return apiRetStatus;
}

aditof::Status CalibrationChicony006::AfeRegRead(uint16_t* punRcvBuf, uint32_t ulWords, uint16_t unRegAdr)
{
    using namespace aditof;
    
    aditof::Status apiRetStatus = Status::OK;

    apiRetStatus = m_device->readAfeRegisters((const uint16_t*)&unRegAdr,
        punRcvBuf, 1);

	return apiRetStatus;
}

/*
	************************** Read EEPROM **********************
	check EEPROM version -> common registers -> each mode registers-> read CS power-up sequence -> [2nd] checksum

*/

// Read 4 types data frpm EEPROM
// [1] common registers
// [2] Mode0 registers
// [3] Mode1 registers
// [4] power up sequence data
aditof::Status CalibrationChicony006::ReadCalibData(void)
{
    using namespace aditof;
    
    aditof::Status	apiRetStatus = Status::OK;
	uint16_t unROMByte=0,unData=0;
	uint16_t unCheckSum = 0;

	/* [1st] check EEPROM version */
	EepromRead(ROM_CMN_BASE+ROM_MAP_VERSION, &unData);
	if( (unData & 0xFF00) != 0x3000 && (unData & 0xFF00) != 0x3100 ){
		LOG(WARNING) << "Error! This EEPROM version (0x%x) is not supported " << unData;
		return Status::GENERIC_ERROR;
	}

	/*** common registers **************************************/
	apiRetStatus = ReadCommonfromEEPROM(&unCheckSum, gunCtrlCmn,  gunIspRegCmn, ROM_CMN_BASE);

	/*** each mode registers ***********************************/
	apiRetStatus = ReadModefromEEPROM(&unCheckSum, gunCtrlMode[0], gunIspRegMode[0], gunIspRamMode[0], ROM_MODE0_BASE);
	apiRetStatus = ReadModefromEEPROM(&unCheckSum, gunCtrlMode[1], gunIspRegMode[1], gunIspRamMode[1], ROM_MODE1_BASE);

	/*** read CS power-up sequence (compressed data)*************************************/
	unROMByte = gunCtrlCmn[CTRL_CMN_CS_INI_CODE_SIZE];
	apiRetStatus = ReadCsIniCodefromEEPROM( unROMByte, &unCheckSum);

	/* [2nd] checksum */
	if( gunCtrlCmn[CTRL_CMN_CHECK_SUM] != unCheckSum ){
		LOG(WARNING) << "Error! EEPROM read failed";
		return Status::GENERIC_ERROR;
	}

	return apiRetStatus;
}

/*  decompression and send CS*/
aditof::Status CalibrationChicony006::DecompressionAndSendCS( void )
{
    using namespace aditof;
    
    unsigned short unCnt;
	uint16_t unCS_Addr=0,unCS_cnt=0,unCS_Size=0,unData;
	eREADCSINICODE_STATUS eStatus = READCSINICODE_SIZE;
	aditof::Status	apiRetStatus = Status::OK;

	for(unCnt = 0 ;unCnt < gunCsTable_Size ; unCnt++){

		unData = gunCsTable[unCnt];
		//LOG(WARNING) << "unCstable 0x%x",unData);

		switch(eStatus){
			case READCSINICODE_SIZE:
				/* read size */
				unCS_Size = unData;

				eStatus = READCSINICODE_ADDR;
				break;
			case READCSINICODE_ADDR:
				/* read start address */
				unCS_Addr = unData;
				unCS_cnt = 0;

				eStatus = READCSINICODE_DATA;
				break;
			case READCSINICODE_DATA:
				/* Write data */
				apiRetStatus = AfeRegWrite(unCS_Addr, unData);

				if( unCS_cnt < unCS_Size-1 ){
					unCS_Addr++;
					unCS_cnt++;
					eStatus = READCSINICODE_DATA;
				}else{
					unCS_cnt = 0;
					eStatus = READCSINICODE_SIZE;
				}
				break;
		}

	}

	return apiRetStatus;
}

aditof::Status CalibrationChicony006::SendCsTable(uint16_t *punSndTbl, uint32_t ulWords)
{
    using namespace aditof;
    
    aditof::Status	apiRetStatus = Status::OK;
	uint32_t  ulLoop = 0;
	uint16_t* punSndCurr;

	punSndCurr = punSndTbl;

	/* Continuously write CS registers */
	for (ulLoop = 0 ; ulLoop < ulWords ; ulLoop++) {
		apiRetStatus = AfeRegWrite(*punSndCurr, *(punSndCurr + 1));
		if (apiRetStatus != Status::OK) {
			return apiRetStatus;
		}
		punSndCurr += 2;
	}

	return apiRetStatus;
}

/*
* Read EEPROM - > power up sequence -> TOF ISP registers ->
* Read image position -> read Coring value -> Set GPO on CS
*/
aditof::Status CalibrationChicony006::initialize(std::shared_ptr<aditof::DeviceInterface> device)
{
    using namespace aditof;
    
    uint16_t unData;
	aditof::Status CyStatus = Status::OK;

    m_device = device;

	/* Read calibration data from EEPROM */
	CyStatus = ReadCalibData();
	if( CyStatus != Status::OK ){
		LOG(WARNING) << "Failed to read calibration data from EEPROM";
		return Status::GENERIC_ERROR;	//!< read error
	}
	LOG(WARNING) << "Read Calibration Data from EEPROM Done";

    /* power up sequence (decompression and send CS)  */
    CyStatus = DecompressionAndSendCS();
    if( CyStatus != Status::OK ){
		LOG(WARNING) << "Failed to program AFE with load data";
		return Status::GENERIC_ERROR;	//!< read error
	}
	LOG(WARNING) << "Programmed AFE with load data";

#if _DEBUG_PRINT_ON
    AfeRegRead(&unData, 1, 0xC76F);
	LOG(WARNING) << "after power-up sequence 0xC76F " << unData;
	LOG(WARNING) << "gunRangeMode    : " << gunRangeMode;
	LOG(WARNING) << "EXP_MAX         : " << gunCtrlMode[gunRangeMode][CTRL_MODE_EXP_MAX];
	LOG(WARNING) << "EXP_DEFAULT     : " << gunCtrlMode[gunRangeMode][CTRL_MODE_EXP_DFLT];
	LOG(WARNING) << "MODE_FLAG       : " << gunCtrlMode[gunRangeMode][CTRL_MODE_TOF_MODE_FLAG];
#endif /* _DEBUG_PRINT_ON */

    /* TOF ISP registers */
   	CyStatus = SendCsTable((uint16_t *)gunIspRegCmn, gunIspRegCmn_Size); /* for all mode */
   	CyStatus = setMode(gunRangeMode);     //init  gunRangeMode = 0, 0 is short 1 : long mode       /* for current mode */
	if( CyStatus != Status::OK ){
		LOG(WARNING) << "Error! check phase parameters";
		return CyStatus;
	}

#if _DEBUG_PRINT_ON
    AfeRegRead(&unData, 1, 0xC76F);
    LOG(WARNING) << "after change Range Mode 0xC76F " << unData;
#endif


    /******* Set valuables **********/
	/* Read image position */
    uint16_t gunPosX, gunPosY;
    AfeRegRead(&gunPosX, 1, 0xC3D3);
	AfeRegRead(&gunPosY, 1, 0xC3D4);

	/* read Coring value */
    uint16_t gunCoring;
    AfeRegRead(&gunCoring, 1, 0xC34A);
    gunCoring &= 0x3FFF;

    /****** Set GPO on CS ************/
    AfeRegWrite(0xC08E, 0x0004);	//GPO3:FET-Driver Enable

    //select Depth, IT mode format
    AfeRegWrite(gunIspRegMode[gunRangeMode][ISP_REG_MODE_ALIGN6][0], gunIspRegMode[gunRangeMode][ISP_REG_MODE_ALIGN6][1]);
    AfeRegWrite(gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI0][0], gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI0][1]);
    AfeRegWrite(gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI1][0], gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI1][1]);   //ROI1(TOF_RAW_ROI_HSTART) 170424
    AfeRegWrite(gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI2][0], gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI2][1]);   //ROI2(TOF_RAW_ROI_VSTART) 170424
    AfeRegWrite(gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI3][0], gunIspRegMode[gunRangeMode][ISP_REG_MODE_ROI3][1]);
    AfeRegWrite(gunIspRegCmn[ISP_REG_CMN_OUTPUTSEL][0], gunIspRegCmn[ISP_REG_CMN_OUTPUTSEL][1]);								 //OUTPUT_SEL bit1 1:IR with BG 0:IR , bit0 0:Depth ,1:BG
    AfeRegWrite(0xC08F, 0x0040);
    AfeRegWrite(0xC087, 0x0027);
    AfeRegWrite(gunIspRegMode[gunRangeMode][ISP_REG_MODE_READ_SIZE5][0], gunIspRegMode[gunRangeMode][ISP_REG_MODE_READ_SIZE5][1]);   // READSIZE5
    LOG(WARNING) << "probe 640x960";

    //set output selection to IR + DEPTH
    uint16_t afeRegsAddr[3] = { 0xc3da, 0x4001, 0x7c22 };
    uint16_t afeRegsVal[3] = { 0x07, 0x0007, 0x0004 };
    m_device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 3);

    return CyStatus;
}

//! setMode - Sets the mode to be used for depth calibration
/*!
setMode - Sets the mode to be used for depth calibration
\param mode - Camera depth mode
*/
aditof::Status CalibrationChicony006::setMode(uint16_t unMode)
{
    using namespace aditof;
    
    aditof::Status	apiRetStatus = Status::OK;
    uint16_t unData, unData2;
    sTOF_PULSE_PARAM sTofParam;
    uint16_t unTalFlag, unLdFlag;
    uint16_t unHdNum;

    /////////// set target Mode
    gunRangeMode = unMode;

    /////////////// Set TAL Flag
    unTalFlag = (gunCtrlCmn[CTRL_CMN_TAL_MODE_FLAG] & 0x000F); //TAL is not change each mode.

#if _DEBUG_PRINT_ON
    LOG(WARNING) << "TAL Flag addr: " << gunCtrlCmn[CTRL_CMN_TAL_MODE_FLAG] << " val: " << unTalFlag;
#endif

#if _DEBUG_SETTING_OFF==0
    // TAL mode check
    if (unTalFlag != 0x0000) {
        //if TAL mode on, stop TAL.
        AfeRegWrite(0xc75E, 0x0000);	//TAL-disable
        AfeRegWrite(0xc730, 0x0000);	//TAL-invalid
    }
#endif

    //////////// Get LD Flag
    unLdFlag = 0;
    if ((gunCtrlMode[gunRangeMode][CTRL_MODE_LD_FLAG] & 0x000F) == 1) {
        unLdFlag |= 0x0001;
    }
    if (((gunCtrlMode[gunRangeMode][CTRL_MODE_LD_FLAG] & 0x00F0) >> 4) == 1) {
        unLdFlag |= 0x0002;
    }
    if (((gunCtrlMode[gunRangeMode][CTRL_MODE_LD_FLAG] & 0x0F00) >> 8) == 1) {
        unLdFlag |= 0x0004;
    }
    if (((gunCtrlMode[gunRangeMode][CTRL_MODE_LD_FLAG] & 0xF000) >> 12) == 1) {
        unLdFlag |= 0x0008;
    }
#if _DEBUG_PRINT_ON
    LOG(WARNING) << "LdFlag addr: " << gunCtrlMode[gunRangeMode][CTRL_MODE_LD_FLAG] << " val: " << unLdFlag;
#endif

    //////////// set ISP Registers
    switch (unMode) {
    case 1:
        apiRetStatus = SendCsTable((uint16_t *)gunIspRegMode[unMode], gunIspRegMode1_Size);
        break;
    case 0:
    default:
        apiRetStatus = SendCsTable((uint16_t *)gunIspRegMode[unMode], gunIspRegMode0_Size);
        break;
    }

    //////////// set VD Duration
    unHdNum = gunCtrlMode[unMode][CTRL_MODE_VD_DURATION] - 2;
    AfeRegWrite(gunIspRamMode[unMode][ISP_RAM_MODE_VD_REG_ADR], unHdNum);

    /////////// set VD initial offset
    gunVdInitialOffset = gunCtrlMode[unMode][CTRL_MODE_VD_INI_OFST];
    SetExposureDelay(unMode, gunVdInitialOffset);
    GetExposureDelay(unMode, &gunVdInitialOffset);


    ////////////// update coring value of internal variable
    uint16_t gunCoring;
    AfeRegRead(&gunCoring, 1, 0xC34A);
    gunCoring &= 0x3FFF;

    ////////////// update pulse parameters of internal variable
    // Get TOF pulse
    apiRetStatus = GetTofPulse(&sTofParam);
    if (apiRetStatus != Status::OK) {
        //set PhaseError.fail safe.
        return apiRetStatus;
    }

#if _DEBUG_PRINT_ON
    LOG(WARNING) << "LD pulse width : " << sTofParam.unLdWidth;
    LOG(WARNING) << "Sub pulse width : " << sTofParam.unSubWidth;
    LOG(WARNING) << "A0 pulse phase : " << sTofParam.unA0Phase;
    LOG(WARNING) << "A1 pulse phase : " << sTofParam.unA1Phase;
    LOG(WARNING) << "A2 pulse phase : " << sTofParam.unA2Phase;
    LOG(WARNING) << "Sub pulse phase : " << sTofParam.unSubPhase;
#endif /* _DEBUG_PRINT_ON */

#if _DEBUG_SETTING_OFF==0
    ////////////////// set exposure
    gunExpValue = GetDefExpValue(unMode);
    SetExpValue( gunExpValue , &gunHdExp);
    AfeRegWrite( 0xC315 , 0);
    AfeRegWrite( 0xC30E , gunHdExp);	//update Hounds-tooth
    AfeRegWrite( 0xC315 , 1);
    SetCcdDummy( gunCcdDummy );
    LOG(WARNING) << "rExposure : " << gunExpValue;
#endif /* _DEBUG_SETTING_OFF */

    switch (unMode) {
    case 1:
        // CS_INI_CODE mode
        AfeRegWrite(0x4000, 0x0001); // mode 1
        break;
#if 0
    case 2:
        // CS_INI_CODE mode
        AfeRegWrite(0x4000, 0x0002); // mode 2
        break;
    case 3:
        // CS_INI_CODE mode
        AfeRegWrite(0x4000, 0x0003); // mode 3
        break;
#endif
    case 0:
    default:
        // CS_INI_CODE mode
        AfeRegWrite(0x4000, 0x0000); // mode 0
        break;
    }

    ///////////////set LD
    unData = 0x0010;
    unData |= (unLdFlag & 0x000F);
    AfeRegWrite(0xc084, unData);	//set LD register1
    AfeRegWrite(0xc085, unData);	//set LD register2
#if _DEBUG_PRINT_ON
    LOG(WARNING) << "ChangeMode LdFlag " << unData;
#endif /* _DEBUG_PRINT_ON */


#if _DEBUG_SETTING_OFF==0
    //TAL mode check
    if (unTalFlag != 0x0000) {
        //if TAL-mode , TAL restart.
        unData2 = unData | (unData << 8);
#if _DEBUG_PRINT_ON
        LOG(WARNING) << "0xC730 " << unData2;
        LOG(WARNING) << "TAL flag " << unTalFlag;
#endif /* _DEBUG_PRINT_ON */
        //select TAL valid edge
        // C730
        //	0xF000 is SUB rising Edge  (start)
        //	0x0F00 is LD  falling edge (start)
        //  0x00F0 is SUB falling edge (end)
        //	0x000F is LD  rising edge  (end)
        if ((unTalFlag & 0x0001) == 0x0001) {
            // LD rising edge TAL-valid
        }
        else {
            // LD rising edge TAL-invalid
            unData2 &= 0xFFF0;
        }

        if ((unTalFlag & 0x0002) == 0x0002) {
            // LD falling edge TAL-valid
        }
        else {
            // LD falling edge TAL-invalid
            unData2 &= 0xF0FF;
        }

        if ((unTalFlag & 0x0004) == 0x0004) {
            // SUB rising edge TAL-valid
        }
        else {
            // SUB rising edge TAL-invalid
            unData2 &= 0x0FFF;
        }

        if ((unTalFlag & 0x0008) == 0x0008) {
            // SUB falling edge TAL-valid
        }
        else {
            // SUB falling edge TAL-invalid
            unData2 &= 0xFF0F;
        }

#if _DEBUG_PRINT_ON
        LOG(WARNING) << "TAL valid-edge : 0xC730 " << unData2;
#endif
        AfeRegWrite(0xc730, unData2);	//TAL-valid (set LD register for TAL)
        AfeRegWrite(0xc75E, unData);	//TAL-enable (set LD register for TAL)

#if _DEBUG_PRINT_ON
        LOG(WARNING) << "TChangeMode LdFlag " << unData;
#endif
    }
#endif /* _DEBUG_SETTING_OFF */

    return apiRetStatus;
}
