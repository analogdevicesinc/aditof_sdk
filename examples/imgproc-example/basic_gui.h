#ifndef BASICGUI_H
#define BASICGUI_H

#include <aditof/frame.h>
#include <fstream>
#include <string>

#include "basic_controller.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif

class Basic_GUI {
  public:
    Basic_GUI(const std::string &name);
    ~Basic_GUI();

    void setFrame(aditof::Frame capturedframe);
    void renderOnce();
    int renderCyclic();

    void renderDepthImage();
    void renderIRImage();
    void detectEdgesfromDepth();
    void detectEdgesfromDepthImage();
    void setDistanceValue(int distanceVal);

  private:
    cv::Mat computeUdisparity(cv::Mat ImgToProcess, int max_disp);
    cv::Mat computeVdisparity(cv::Mat ImgToProcess, int max_disp);
    cv::Mat computeThreshold(cv::Mat ImgToProcess, int const_value);

  private:
    std::shared_ptr<aditof::Frame> m_capturedFrame;
    std::string m_viewName;

    cv::Mat m_depthImage;
    cv::Mat m_irImage;

    const std::string WINDOW1_NAME = "Depth Image";
    const std::string WINDOW2_NAME = "IR Image";
    const std::string WINDOW3_NAME = "Edges";
    const std::string WINDOW4_NAME = "Histogram";
    const std::string WINDOW5_NAME = "Detected Objects";

    const int MINSIZEOFBLOB_NEAR = 40;

    int m_distanceVal;

    int m_renderDepth;
    int m_renderIR;
    int m_detectEdges;
    int m_picture_work;

    int m_Theta1;
    int m_Theta2;
};

#endif
