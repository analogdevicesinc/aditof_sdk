#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <aditof/camera.h>
#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include "safequeue.h"

class ImgProc_Controller {

  public:
    ImgProc_Controller();
    ~ImgProc_Controller();

    void startCapture(/*unsigned int whichCamera*/);
    void stopCapture();

    std::string getMode() const;
    void setMode(const std::string &mode);

    aditof::Status writeAFEregister(uint16_t *address, uint16_t *data,
                                    uint16_t noOfEntries = 1);
    aditof::Status readAFEregister(uint16_t *address, uint16_t *data,
                                   uint16_t noOfEntries = 1);

    std::shared_ptr<aditof::Frame> getFrame();
    void requestFrame();

    bool hasCamera() const;

  private:
    void captureFrames();

  private:
    aditof::System m_system;
    std::vector<aditof::Camera *> m_cameras;
    int m_cameraInUse;
    std::thread m_workerThread;
    std::atomic<bool> m_stopFlag;
    SafeQueue<std::shared_ptr<aditof::Frame>> m_queue;
    std::mutex m_mutex;
    std::mutex m_requestMutex;
    std::condition_variable m_requestCv;
    bool m_frameRequested;
};

#endif
