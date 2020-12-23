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
#ifndef ADITOFDEMOCONTROLLER_H
#define ADITOFDEMOCONTROLLER_H

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>

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

    int getRangeMax() const;

    int getRangeMin() const;

    int getbitCount() const;

    bool setNetworkConnection(const std::string &ip);
    bool setRegularConnection();

    aditof::Status enableNoiseReduction(bool en);
    aditof::Status setNoiseReductionThreshold(uint16_t threshold);
    aditof::Status setIrGammaCorrection(float gamma);

    aditof::Status setCameraRevision(const std::string &revision);

  private:
    void captureFrames();

  private:
    aditof::System *m_system;
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

    bool m_IsNetworkConnection = false;
};

#endif
