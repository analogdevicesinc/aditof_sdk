/********************************************************************************/
/*																				*/
/* @file	NRFiter3D.cpp
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
/* Copyright(c) 2017, Analog Devices, Inc. All rights reserved. */
/*																				*/
/********************************************************************************/
#include "NRFilter3D.h"
#include "omp.h"
#include "types.h"
#include <cstdlib>
#include <math.h>

/*
 * Gaus5x5 - Optimized Gaussian 5x5 filter.
 *   pnDst - input/output image
 *   iW - Image width
 *   iH - Image height
 */
void FltGaus5x5(uint16 *pnDst, uint16 *pnSrc, int iW, int iH) {
    int sumA5, sumB3;

    int *sumA1 = (int *)calloc(iW, sizeof(int));
    int *sumA2 = (int *)calloc(iW, sizeof(int));
    int *sumA3 = (int *)calloc(iW, sizeof(int));
    int *sumA4 = (int *)calloc(iW, sizeof(int));
    int *sumB1 = (int *)calloc(iW, sizeof(int));
    int *sumB2 = (int *)calloc(iW, sizeof(int));

    for (int j = 0; j < iH - 2; j++) {
        int trmA1 = 0, trmA2 = 0, trmA3 = 0, trmA4 = 0, trmA5;
        int trmB1 = 0, trmB2 = 0, trmB3;
        int y = j * iW;
        //#pragma omp parallel for
        for (int i = 0; i < iW - 2; i++) {
            trmB3 = 24 * pnSrc[y + (i + 1)];
            trmA5 = 6 * pnSrc[y + (i + 2)];
            int y1 = y + iW;
            sumB3 = 4 * pnSrc[y1 + (i - 2)] + 16 * pnSrc[y1 + (i - 1)] +
                    24 * pnSrc[y1 + i] + 16 * pnSrc[y1 + (i + 1)] +
                    4 * pnSrc[y1 + (i + 2)];
            int y2 = y1 + iW;
            sumA5 = pnSrc[y2 + (i - 2)] + 4 * pnSrc[y2 + (i - 1)] +
                    6 * pnSrc[y2 + i] + 4 * pnSrc[y2 + (i + 1)] +
                    pnSrc[y2 + (i + 2)];

            pnDst[y + i] =
                (sumA1[i] + sumB1[i] + trmA1 + trmB1 + 36 * pnSrc[y + i] +
                 trmB3 + trmA5 + sumA5 + sumB3 + 128) >>
                8;

            trmA1 = trmA2;
            trmA2 = trmA3;
            trmA3 = trmA4;
            trmA4 = trmA5;
            trmB1 = trmB2;
            trmB2 = trmB3;
            sumA1[i] = sumA2[i];
            sumA2[i] = sumA3[i];
            sumA3[i] = sumA4[i];
            sumA4[i] = sumA5;
            sumB1[i] = sumB2[i];
            sumB2[i] = sumB3;
        }
    }

    free(sumA1);
    free(sumA2);
    free(sumA3);
    free(sumA4);
    free(sumB1);
    free(sumB2);
}

NRFilter3D::NRFilter3D(uint16 unHeight, uint16 unWidth, uint16 *punDepth,
                       uint16 *punLastDepth) {
    this->unHeight = unHeight;
    this->unWidth = unWidth;
    this->punDepth = punDepth;
    this->punLastDepth = punLastDepth;

    // this->pMVec = NULL;
    // this->punMVComp = NULL;
    this->punDFilt = NULL;
    this->puiPrjLRow = NULL;
    this->puiPrjLCol = NULL;
    this->puiPrjNRow = NULL;
    this->puiPrjNCol = NULL;

#ifdef FIVE_FRAME_AVG
    iIdx = 0;
    for (int i = 0; i < 5; i++) {
        this->unBuf[i] = (uint16 *)malloc(unHeight * unWidth * sizeof(uint16));
    }
#endif
}

NRFilter3D::~NRFilter3D() {}

/*
 * init - Allocates all the internal buffers.
 * Called once before calling nrfilter3d()
 */
void NRFilter3D::init(void) {
    int iBufSize = this->unHeight * this->unWidth;
    // this->pMVec = (mvec_t *) calloc(iBufSize, sizeof(mvec_t));
    this->punDFilt = (uint16 *)malloc(iBufSize * sizeof(uint16));
    // this->punMVComp = (uint16 *) malloc(iBufSize*sizeof(uint16));

    this->puiPrjLRow = (uint32 *)calloc(iBufSize, sizeof(int));
    this->puiPrjLCol = (uint32 *)calloc(iBufSize, sizeof(int));
    this->puiPrjNRow = (uint32 *)calloc(iBufSize, sizeof(int));
    this->puiPrjNCol = (uint32 *)calloc(iBufSize, sizeof(int));
}

/*
 * release - Frees all the internal buffers.
 * Called once after nrfilter3d() is no longer called.
 */
void NRFilter3D::release(void) {
    // if (this->pMVec) {
    //	free(this->pMVec);
    //}
    // if (this->punMVComp) {
    //	free(this->punMVComp);
    //}
    if (this->punDFilt) {
        free(this->punDFilt);
    }
    if (this->puiPrjLRow) {
        free(this->puiPrjLRow);
    }
    if (this->puiPrjLCol) {
        free(this->puiPrjLCol);
    }
    if (this->puiPrjNRow) {
        free(this->puiPrjNRow);
    }
    if (this->puiPrjNCol) {
        free(this->puiPrjNCol);
    }
}
