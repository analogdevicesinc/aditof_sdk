#ifndef ADITOFDEMORECORDER_H
#define ADITOFDEMORECORDER_H
#include <frame.h>

#include <atomic>
#include <fstream>
#include <thread>

#include "safequeue.h"

class AditofDemoRecorder {
  public:
    AditofDemoRecorder();
    ~AditofDemoRecorder();

    void startRecording(const std::string &fileName, unsigned int height,
			unsigned int width, unsigned int fps);
    void stopRecording();

    int startPlayback(const std::string &fileName, int &fps);
    void stopPlayback();

    void recordNewFrame(std::shared_ptr<aditof::Frame> frame);
    std::shared_ptr<aditof::Frame> readNewFrame();

    void requestFrame();

    bool isRecordingEnabled() const;
    bool isPlaybackEnabled() const;

    bool isPlaybackFinished() const;

    int getNumberOfFrames() const;

  private:
    void recordThread();
    void playbackThread();

  private:
    SafeQueue<std::shared_ptr<aditof::Frame>> m_recordQueue;
    SafeQueue<std::shared_ptr<aditof::Frame>> m_playbackQueue;

    std::ofstream m_recordFile;
    std::ifstream m_playbackFile;

    aditof::FrameDetails m_frameDetails;

    std::thread m_recordThread;
    std::thread m_playbackThread;
    std::atomic<bool> m_recordTreadStop;
    std::atomic<bool> m_playbackThreadStop;
    bool m_shouldReadNewFrame;
    std::mutex m_playbackMutex;
    std::condition_variable m_playbackCv;
    bool m_playBackEofReached;

    int m_numberOfFrames;
};

#endif // ADITOFDEMORECORDER_H
