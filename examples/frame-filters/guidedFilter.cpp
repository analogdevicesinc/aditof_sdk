/********************************************************************************/
/*																				*/
/* @file	guidedFilter.cpp
 */
/*																				*/
/* @brief	Flying pixel filter
 */
/*																				*/
/* @author	Charles Mathy, Dhruvesh Gajaria
 */
/*																				*/
/* @date	March 22, 2017
 */
/*																				*/
/* Copyright(c) Analog Devices, Inc.
 */
/*																				*/
/********************************************************************************/

#include "guidedFilter.h"

void guidedFilter(cv::Mat &inS0, cv::Mat &inS1, int r, float eps) {
    cv::Mat temp;
    cv::Mat S0;
    cv::Mat S1;

    inS0.convertTo(S0, CV_32FC1);
    inS1.convertTo(S1, CV_32FC1);

    cv::Mat meanS0;
    cv::blur(S0, meanS0, cv::Size(r, r));

    cv::Mat meanS1;
    cv::blur(S1, meanS1, cv::Size(r, r));

    cv::Mat meanS0S0 = meanS0.mul(meanS0);
    cv::Mat meanS1S1 = meanS1.mul(meanS1);

    cv::Mat S0S0 = S0.mul(S0);
    cv::Mat S1S1 = S1.mul(S1);

    cv::blur(S0S0, temp, cv::Size(r, r));
    cv::Mat varS0 = temp - meanS0S0;

    cv::blur(S1S1, temp, cv::Size(r, r));
    cv::Mat varS1 = temp - meanS1S1;

    cv::Mat den = varS0 + varS1 + eps;

    cv::Mat alpha;
    divide((varS0 + varS1), den, alpha);

    cv::Mat beta0;
    beta0 = meanS0.mul(eps);
    divide(beta0, den, beta0);

    cv::Mat beta1;
    beta1 = meanS1.mul(eps);
    divide(beta1, den, beta1);

    cv::blur(alpha, alpha, cv::Size(r, r));
    cv::blur(beta0, beta0, cv::Size(r, r));
    cv::blur(beta1, beta1, cv::Size(r, r));

    cv::Mat S0_filt = S0.mul(alpha) + beta0;
    cv::Mat S1_filt = S1.mul(alpha) + beta1;

    double minVal, maxVal;
    minMaxLoc(S0_filt, &minVal, &maxVal); // find minimum and maximum
                                          // intensities
    S0_filt.convertTo(S0_filt, CV_16UC1);

    minMaxLoc(S1_filt, &minVal, &maxVal); // find minimum and maximum
                                          // intensities
    S1_filt.convertTo(S1_filt, CV_16UC1);

    S0_filt.copyTo(inS0);
    S1_filt.copyTo(inS1);

    temp.release();
    S0.release();
    S1.release();
    S0S0.release();
    S1S1.release();
    meanS0.release();
    meanS1.release();
    meanS0S0.release();
    meanS1S1.release();
    varS0.release();
    varS1.release();
    den.release();
    alpha.release();
    beta0.release();
    beta1.release();
    S0_filt.release();
    S1_filt.release();
}

cv::Mat guidedDepthFilter(const cv::Mat &inDepth, int filterSize, float eps) {
    cv::Mat outDepth;
    inDepth.convertTo(outDepth, CV_32FC1);

    cv::Mat muI;
    cv::blur(outDepth, muI, cv::Size(filterSize, filterSize));

    cv::Mat muI2;
    muI2 = muI.mul(muI);

    cv::Mat IxP;
    cv::blur(outDepth.mul(outDepth), IxP, cv::Size(filterSize, filterSize));

    cv::Mat sigmaISq;
    sigmaISq = IxP - muI2;

    cv::Mat ak;
    ak = (IxP - muI2);
    divide(ak, (sigmaISq + eps), ak);

    cv::Mat bk;
    bk = muI - ak.mul(muI);

    cv::Mat a;
    cv::blur(ak, a, cv::Size(filterSize, filterSize));

    cv::Mat b;
    cv::blur(bk, b, cv::Size(filterSize, filterSize));

    outDepth = outDepth.mul(a) + b;

    double minVal, maxVal;
    cv::minMaxLoc(outDepth, &minVal,
                  &maxVal); // find minimum and maximum intensities
    outDepth.convertTo(outDepth, CV_8U);

    return outDepth;
}
