#ifndef HELLOWORLDVIEW_H
#define HELLOWORLDVIEW_H

#include <aditof/frame.h>
#include <fstream>
#include <string>

#include "aditofdemocontroller.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif

class AdiTofDemoView {
  public:
    AdiTofDemoView(std::shared_ptr<AdiTofDemoController> &ctrl,
                   const std::string &name);
    ~AdiTofDemoView();

    void render();

  private:
    void _displayDepthImage();
    void _displayIrImage();
    void _displayBlendedImage();

  private:
    std::shared_ptr<AdiTofDemoController> m_ctrl;
    std::string m_viewName;

    cv::Mat m_depthImage;
    cv::Mat m_irImage;
    cv::Mat m_blendedImage;
    double m_blendValue = 0.5;

    std::thread m_depthImageWorker;
    std::thread m_irImageWorker;
    std::mutex m_frameCapturedMutex;
    bool m_depthFrameAvailable;
    bool m_irFrameAvailable;
    std::shared_ptr<aditof::Frame> m_capturedFrame;
    std::condition_variable m_frameCapturedCv;
    bool m_stopWorkersFlag;
    bool m_center;

    std::mutex m_imshowMutex;
    int m_waitKeyBarrier;
    std::condition_variable m_barrierCv;
    int m_distanceVal;

    bool m_smallSignal;
    bool m_crtSmallSignalState;
};

#endif
