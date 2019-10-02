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
    void initRendering();
    int renderCyclic();
    int getMode();
    void renderDepthImage();
    void setRange(int range);

  private:
    void applyVarianceFilter();
    void applyFlyingPixelFilter();
    void applyGuidedFilter();
    void applyNR3DFilter();
    void applyTimeFilter();

  private:
    std::shared_ptr<aditof::Frame> m_capturedFrame;
    std::string m_viewName;
    int m_range = 59999;

    cv::Mat m_depthImage;

    const std::string WINDOW_DEPTH = "Depth Image";
    const std::string WINDOW_VARIANCE_F = "Variance filter";
    const std::string WINDOW_GUIDED_F = "Guided filter";
    const std::string WINDOW_FLYINGPIXEL_F = "Flying Pixel filter";
    const std::string WINDOW_FILTER4 = "Filter 4";

    bool m_renderDepth;
    bool m_varianceFilter;
    bool m_guidedFilter;
    bool m_flyingPixelFilter;

    int m_filterSize = 3;
    int m_threshold = 10;
    double m_eps = 10.0;
    double m_edgeThi = 2.0;
    double m_epsEdgei = 2.0;

    int m_filterSizeXi = 3;
    int m_filterSizeYi = 3;

    bool m_checkboxChanged = false;
    bool m_nearModeChecked = false;
    bool m_mediumModeChecked = false;
    bool m_farModeChecked = true;
    int m_modeCurrentValue = 1;
};

#endif
