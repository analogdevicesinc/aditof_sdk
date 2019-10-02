/********************************************************************************/
/*																				*/
/* @file	NRFiter3D.h
 */
/*																				*/
/* @brief	Performs motion compensated temporal filtering.
 */
/*																				*/
/* @author	Rick Haltmaier
 */
/*																				*/
/* @date	June 23, 2016
 */
/*																				*/
/* Copyright(c) Analog Devices, Inc.
 */
/*																				*/
/********************************************************************************/
#ifndef _NRFILTER3D_H_
#define _NRFILTER3D_H_

#include "types.h"

typedef struct {
    int x;
    int y;
} mvec_t;

class NRFilter3D {
    uint16 unHeight;
    uint16 unWidth;
    uint16 *punDepth;
    uint16 *punLastDepth;
    // mvec_t *pMVec;
    // uint16 *punMVComp;
    uint16 *punDFilt;

    // Row and Column projection buffers for new and last depth
    uint32 *puiPrjLRow;
    uint32 *puiPrjLCol;
    uint32 *puiPrjNRow;
    uint32 *puiPrjNCol;

    // int MadCalc(uint16 *punA, uint16 *punB, int iWidth);
    // void StepSearch(int x, int y, int *iVecX, int *iVecY, int step);
    // void StepSearch(int x, int y, uint32 *puiPrjLRow, uint32 *puiPrjLCol,
    // uint32 *puiPrjNRow, uint32 *puiPrjNCol, int *iVecX, int *iVecY, int
    // step);
    int MadCalcR(uint32 *punA, uint32 *punB, int iWidth);
    int MadCalcC(uint32 *punA, uint32 *punB);
    // void MVSearch(mvec_t *pMVec);

#ifdef FIVE_FRAME_AVG
    int iIdx;
    uint16 *unBuf[5];
#endif

  public:
    NRFilter3D(uint16 unHeight = 480, uint16 unWidth = 640,
               uint16 *punDepth = NULL, uint16 *punLastDepth = NULL);
    ~NRFilter3D();
    void init(void);
    void release(void);
    void setDimensions(uint16 unHeight, uint16 unWidth) {
        this->unHeight = unHeight;
        this->unWidth = unWidth;
    }
    void setDepthPtr(uint16 *punDepth) { this->punDepth = punDepth; }
    void setLastDepthPtr(uint16 *punLastDepth) {
        this->punLastDepth = punLastDepth;
    }

    int nrfilter3d(uint8 ucAlpha);

    // Debug stats
    int iStat;
    int iTotal;
};

#endif // _NRFILTER3D_H_
