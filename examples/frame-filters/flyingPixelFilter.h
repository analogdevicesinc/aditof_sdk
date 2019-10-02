/********************************************************************************/
/*																				*/
/* @file	flyingPixelFilter.cpp
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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
using namespace std;

class flyingPixelFilter {

    int height, width;
    int filterSizeX, filterSizeY;
    float edgeTh;
    float epsEdge;
    cv::Mat *xs, *ys;
    cv::Mat *muD;
    cv::Mat *muxmuDs;
    cv::Mat *muymuDs;
    cv::Mat *xDsSq;
    cv::Mat *yDsSq;
    cv::Mat *xDsTemp;
    cv::Mat *yDsTemp;
    cv::Mat *xDs;
    cv::Mat *yDs;
    cv::Mat *edgesIn;
    cv::Mat *mux;
    cv::Mat *muy;
    cv::Mat *xxs;
    cv::Mat *xxsSq;
    // cv::Mat* edges;

  public:
    flyingPixelFilter(int heighti, int widthi, int filterSizeXi,
                      int filterSizeYi, double edgeThi, double epsEdgei);
    ~flyingPixelFilter();
    void filter(cv::Mat &inDepthSrc, cv::Mat &edgesSrc);
};
