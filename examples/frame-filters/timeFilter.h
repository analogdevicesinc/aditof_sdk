/********************************************************************************/
/*																				*/
/* @file	timeFilter.cpp
 */
/*																				*/
/* @brief	Temporal filter that resets when change is large enough
 */
/*																				*/
/* @author	Charles Mathy, Dhruvesh Gajaria */
/*																				*/
/* @date	March 21, 2017
 */
/*																				*/
/* Copyright(c) Analog Devices, Inc.
 */
/*																				*/
/********************************************************************************/
#include "types.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
//#include <opencv2/contrib/contrib.hpp>
//#include "opencv2/legacy/legacy.hpp"

class temporalFilter {
    int height, width;
    float lambda;
    int nrElts;
    float *_mu0, *_mu1, *_mu2;
    float *mu0, *mu1, *mu2;
    float *_var0, *_var1, *_var2;
    float *_var0N, *_var1N, *_var2N;
    float *var0, *var1, *var2;
    float *S0, *S1, *S2;
    int *frameId;

  public:
    float *stdDev0, *stdDev1, *stdDev2;
    temporalFilter(int heighti, int widthi, float lambdai);
    ~temporalFilter();
    void filter(uint16 *S0, uint16 *S1 = NULL, uint16 *S2 = NULL);
};
