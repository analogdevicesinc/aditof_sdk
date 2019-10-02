/********************************************************************************/
/*																				*/
/* @file	timeFilter.cpp
 */
/*																				*/
/* @brief	Temporal filter that resets when change is large enough
 */
/*																				*/
/* @author	Charles Mathy, Dhruvesh Gajaria
 */
/*																				*/
/* @date	March 21, 2017 */
/*																				*/
/* Copyright(c) Analog Devices, Inc.
 */
/*																				*/
/********************************************************************************/
#include "timeFilter.h"
#include <string.h>

void initializeFloatArray(float *in, int nrElts, float val) {
    for (int i = 0; i < nrElts; i++) {
        in[i] = val;
    }
}

void initializeIntArray(int *in, int nrElts, int val) {
    for (int i = 0; i < nrElts; i++) {
        in[i] = val;
    }
}

float fabss(float x) { return (x > 0. ? x : -x); }

temporalFilter::temporalFilter(int heighti, int widthi, float lambdai)
    : height(heighti), width(widthi), lambda(lambdai) {

    nrElts = height * width;
    _mu0 = new float[nrElts];
    _mu1 = new float[nrElts];
    _mu2 = new float[nrElts];

    mu0 = new float[nrElts];
    mu1 = new float[nrElts];
    mu2 = new float[nrElts];

    _var0 = new float[nrElts];
    _var1 = new float[nrElts];
    _var2 = new float[nrElts];

    _var0N = new float[nrElts];
    _var1N = new float[nrElts];
    _var2N = new float[nrElts];

    var0 = new float[nrElts];
    var1 = new float[nrElts];
    var2 = new float[nrElts];

    stdDev0 = new float[nrElts];
    stdDev1 = new float[nrElts];
    stdDev2 = new float[nrElts];

    S0 = new float[nrElts];
    S1 = new float[nrElts];
    S2 = new float[nrElts];

    frameId = new int[nrElts];

    initializeFloatArray(_mu0, nrElts, 0.);
    initializeFloatArray(_mu1, nrElts, 0.);
    initializeFloatArray(_mu2, nrElts, 0.);
    initializeFloatArray(mu0, nrElts, 0.);
    initializeFloatArray(mu1, nrElts, 0.);
    initializeFloatArray(mu2, nrElts, 0.);
    initializeFloatArray(_var0, nrElts, 0.);
    initializeFloatArray(_var1, nrElts, 0.);
    initializeFloatArray(_var2, nrElts, 0.);
    initializeFloatArray(_var0N, nrElts, 0.);
    initializeFloatArray(_var1N, nrElts, 0.);
    initializeFloatArray(_var2N, nrElts, 0.);
    initializeFloatArray(var0, nrElts, 0.);
    initializeFloatArray(var1, nrElts, 0.);
    initializeFloatArray(var2, nrElts, 0.);
    initializeFloatArray(stdDev0, nrElts, 0.);
    initializeFloatArray(stdDev1, nrElts, 0.);
    initializeFloatArray(stdDev2, nrElts, 0.);
    initializeFloatArray(S0, nrElts, 0.);
    initializeFloatArray(S1, nrElts, 0.);
    initializeFloatArray(S2, nrElts, 0.);
    initializeIntArray(frameId, nrElts, 0);

    // memset(frameId, 0, sizeof(int) * nrElts);
}

temporalFilter::~temporalFilter() {
    free(_mu0);
    _mu0 = NULL;

    free(_mu1);
    _mu1 = NULL;

    free(_mu2);
    _mu2 = NULL;

    free(mu0);
    mu0 = NULL;

    free(mu1);
    mu1 = NULL;

    free(mu2);
    mu2 = NULL;

    free(_var0);
    _var0 = NULL;

    free(_var1);
    _var1 = NULL;

    free(_var2);
    _var2 = NULL;

    free(_var0N);
    _var0N = NULL;

    free(_var1N);
    _var1N = NULL;

    free(_var2N);
    _var2N = NULL;

    free(var0);
    var0 = NULL;

    free(var1);
    var1 = NULL;

    free(var2);
    var2 = NULL;

    free(stdDev0);
    stdDev0 = NULL;

    free(stdDev1);
    stdDev1 = NULL;

    free(stdDev2);
    stdDev2 = NULL;

    free(S0);
    S0 = NULL;

    free(S1);
    S1 = NULL;

    free(S2);
    S2 = NULL;

    free(frameId);
    frameId = NULL;
}

void temporalFilter::filter(uint16 *A0, uint16 *A1, uint16 *A2) {
    for (int i = 0; i < nrElts; i++) {

        frameId[i] += 1;
        double dA0 = (double)A0[i];

        double dMu0 = _mu0[i] + (dA0 - _mu0[i]) / frameId[i];
        double _var0N = _var0[i] + (dA0 - _mu0[i]) * (dA0 - dMu0);
        _mu0[i] = static_cast<float>(dMu0);
        _var0[i] = static_cast<float>(_var0N);
        double dVar0 = _var0N / frameId[i];

        double mu0Diff = 0;
        mu0Diff = dA0 - _mu0[i];
        if (mu0Diff * mu0Diff > lambda * fabss(_mu0[i])) {
            _mu0[i] = static_cast<float>(dA0);
            _var0[i] = 0.;
        }

        A0[i] = (uint16)dMu0;
        stdDev0[i] = sqrtf(static_cast<float>(dVar0));
    }
}
