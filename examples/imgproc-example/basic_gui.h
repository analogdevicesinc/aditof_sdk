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

  private:
    std::shared_ptr<aditof::Frame> m_capturedFrame;
    std::string m_viewName;

    cv::Mat m_depthImage;
    cv::Mat m_depthImageB_W;
    cv::Mat m_irImage;

    const std::string WINDOW1_NAME = "Depth Image";
    const std::string WINDOW2_NAME = "Depth Image B&W";
    const std::string WINDOW3_NAME = "IR Image";

    int m_distanceVal;

    int renderDepth;
};

#endif
