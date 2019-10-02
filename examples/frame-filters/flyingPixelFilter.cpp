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
/*					 															*/
/* Copyright(c) Analog Devices, Inc.
 */
/*																				*/
/********************************************************************************/

#include "flyingPixelFilter.h"

flyingPixelFilter::flyingPixelFilter(int heighti, int widthi, int filterSizeXi,
                                     int filterSizeYi, double edgeThi,
                                     double epsEdgei)
    : height(heighti), width(widthi), filterSizeX(filterSizeXi),
      filterSizeY(filterSizeYi), edgeTh(edgeThi), epsEdge(epsEdgei) {
    xs = new cv::Mat(height, width, CV_32FC1);
    ys = new cv::Mat(height, width, CV_32FC1);
    xDs = new cv::Mat(height, width, CV_32FC1);
    yDs = new cv::Mat(height, width, CV_32FC1);
    edgesIn = new cv::Mat(height, width, CV_32FC1);

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            xs->at<float>(i, j) = (float)(-height / 2 + i) + 0.1;
            ys->at<float>(i, j) = (float)(-width / 2 + j) + 0.1;
        }
    }

    mux = new cv::Mat(height, width, CV_32FC1);
    cv::blur(*xs, *mux, cv::Size(filterSizeX, filterSizeY));
    muy = new cv::Mat(height, width, CV_32FC1);
    cv::blur(*ys, *muy, cv::Size(filterSizeX, filterSizeY));
    cv::Mat xx = (*xs).mul(*xs);
    cv::Mat muxmux = (*mux).mul(*mux);
    xxs = new cv::Mat(height, width, CV_32FC1);
    (*xxs) = xx - muxmux;

    xxsSq = new cv::Mat(height, width, CV_32FC1);
    (*xxsSq) = (*xxs).mul(*xxs);

    muD = new cv::Mat(height, width, CV_32FC1);

    muxmuDs = new cv::Mat(height, width, CV_32FC1);
    muymuDs = new cv::Mat(height, width, CV_32FC1);
    xDsTemp = new cv::Mat(height, width, CV_32FC1);
    yDsTemp = new cv::Mat(height, width, CV_32FC1);

    xDsSq = new cv::Mat(height, width, CV_32FC1);
    yDsSq = new cv::Mat(height, width, CV_32FC1);
}

flyingPixelFilter::~flyingPixelFilter() {
    delete (xs);
    xs = NULL;

    delete (ys);
    ys = NULL;

    delete (xDs);
    xDs = NULL;

    delete (yDs);
    yDs = NULL;

    delete (edgesIn);
    edgesIn = NULL;

    delete (mux);
    mux = NULL;

    delete (mux);
    mux = NULL;

    delete (xxs);
    xxs = NULL;

    delete (xxsSq);
    xxsSq = NULL;

    delete (muD);
    muD = NULL;

    delete (muxmuDs);
    muxmuDs = NULL;

    delete (muymuDs);
    muymuDs = NULL;

    delete (xDsTemp);
    xDsTemp = NULL;

    delete (yDsTemp);
    yDsTemp = NULL;

    delete (xDsSq);
    xDsSq = NULL;

    delete (yDsSq);
    yDsSq = NULL;
}

void flyingPixelFilter::filter(cv::Mat &inDepthSrc, cv::Mat &edgesSrc) {
    cv::Mat inDepth;
    cv::Mat edges;

    inDepthSrc.convertTo(inDepth, CV_32FC1);
    edgesSrc.convertTo(edges, CV_32FC1);

    cv::blur(inDepth, (*muD), cv::Size(filterSizeX, filterSizeY));
    // cv::blur(inDepth,(*muD),cv::Size(r,r));

    (*muxmuDs) = (*muD).mul(*mux);

    (*muymuDs) = (*muD).mul(*muy);

    cv::blur((*xs).mul(inDepth), *xDsTemp, cv::Size(filterSizeX, filterSizeY));
    cv::blur((*ys).mul(inDepth), *yDsTemp, cv::Size(filterSizeX, filterSizeY));

    (*xDs) = (*xDsTemp) - (*muxmuDs);
    (*yDs) = (*yDsTemp) - (*muymuDs);

    (*xDsSq) = (*xDs).mul(*xDs);
    (*yDsSq) = (*yDs).mul(*yDs);

    (*edgesIn) = ((*xDsSq) + (*yDsSq)) / ((*xxsSq) + epsEdge);

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            edges.at<float>(i, j) =
                ((*edgesIn).at<float>(i, j) < edgeTh ? 4096 : 0);
            if (edges.at<float>(i, j) == 0)
                inDepth.at<float>(i, j) = 0;
        }
    }
    inDepth.convertTo(inDepth, CV_8U);
    edges.convertTo(edges, CV_8U);

    inDepth.copyTo(inDepthSrc);
    edges.copyTo(edgesSrc);
}
