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
#ifndef ADITOFDEMOVIEW_H
#define ADITOFDEMOVIEW_H

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

    bool m_crtSmallSignalState;
    bool m_crtIRGamma;
};

#endif
