#ifndef HELLOWORLDCONTROLLER_H
#define HELLOWORLDCONTROLLER_H

#include <camera.h>
#include <frame.h>
#include <system.h>
#include <device_interface.h>

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include "aditofdemorecorder.h"
#include "safequeue.h"

class AdiTofDemoController {

  public:
    AdiTofDemoController();
    ~AdiTofDemoController();

    void startCapture(/*unsigned int whichCamera*/);
    void stopCapture();

    std::string getMode() const;
    void setMode(const std::string &mode);

    std::pair<float, float> getTemperature();

    aditof::Status writeAFEregister(uint16_t *address, uint16_t *data,
                                    uint16_t noOfEntries = 1);
    aditof::Status readAFEregister(uint16_t *address, uint16_t *data,
                                   uint16_t noOfEntries = 1);

    void startRecording(const std::string &fileName, unsigned int height,
			unsigned int width, unsigned int fps);
    void stopRecording();
    int startPlayback(const std::string &fileName, int &fps);
    void stopPlayback();

    bool playbackFinished() const;

    std::shared_ptr<aditof::Frame> getFrame();
    void requestFrame();

    bool hasCamera() const;

  private:
    void captureFrames();

  private:
    std::vector<std::shared_ptr<aditof::Camera>> m_cameras;
    int m_cameraInUse;
    std::thread m_workerThread;
    std::atomic<bool> m_stopFlag;
    SafeQueue<std::shared_ptr<aditof::Frame>> m_queue;
    std::mutex m_mutex;
    std::mutex m_requestMutex;
    std::condition_variable m_requestCv;
    bool m_frameRequested;

    std::unique_ptr<AditofDemoRecorder> m_recorder;
};

#endif
