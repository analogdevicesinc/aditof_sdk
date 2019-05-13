#ifndef HELLOWORLDVIEW_H
#define HELLOWORLDVIEW_H

#include <frame.h>
#include <fstream>
#include <string>

#include "aditofdemocontroller.h"

class AdiTofDemoView {
  public:
    AdiTofDemoView(std::shared_ptr<AdiTofDemoController> &ctrl,
                   const std::string &name);
    ~AdiTofDemoView();

    void render();

  private:
    void _displayDepthImage();
    void _displayIrImage();

  private:
    std::shared_ptr<AdiTofDemoController> m_ctrl;
    std::string m_viewName;

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
};

#endif
