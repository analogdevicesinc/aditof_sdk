#ifndef __TOF_CS_TABLE_H__
#define __TOF_CS_TABLE_H__

#include "tof_isp_table.h"

extern uint16_t gunCsTable[MAX_CS_INI_CODE_SIZE];
extern uint32_t gunCsTable_Size;

typedef enum _eREADCSINICODE_STATUS{
	READCSINICODE_SIZE = 0x0000,
	READCSINICODE_ADDR,
	READCSINICODE_DATA,
}eREADCSINICODE_STATUS;

#endif /* __TOF_CS_TABLE_H__ */
