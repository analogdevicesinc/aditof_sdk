#ifndef BASIC_CONTROLLER_H
#define BASIC_CONTROLLER_H

#include <aditof/camera.h>
#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>

#include <atomic>

class Basic_Controller {

  public:
    Basic_Controller();
    ~Basic_Controller();

  public:
    std::vector<aditof::Frame> captureFrames();
    aditof::Frame getFrame();
    void printFrame(aditof::Frame frame);

  private:
    aditof::System m_system;
    std::vector<aditof::Camera *> m_cameras;
    int m_cameraInUse;
    std::atomic<bool> m_stopFlag;
    std::vector<aditof::Frame> m_frames;

    const int SELECTED_MODE = 2;
};

#endif
