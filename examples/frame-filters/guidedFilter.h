/********************************************************************************/
/*																				*/
/* @file	guidedFilter.h
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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

void guidedFilter(cv::Mat &inS0, cv::Mat &inS1, int r, float eps);

cv::Mat guidedDepthFilter(const cv::Mat &inDepth, int filterSize, float eps);
