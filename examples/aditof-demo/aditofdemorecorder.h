/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ADITOFDEMORECORDER_H
#define ADITOFDEMORECORDER_H
#include <aditof/frame.h>

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
