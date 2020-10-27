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
#include "aditofdemoview.h"

#include <glog/logging.h>
#include <sstream>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

//#define SHOW_ADVANCED_GUI

namespace detail {
std::string getKeyPressed(int key, bool &backspace) {
    switch (key & 0xff) {
    case 46:
        return ".";
    case 48:
        return "0";
    case 49:
        return "1";
    case 50:
        return "2";
    case 51:
        return "3";
    case 52:
        return "4";
    case 53:
        return "5";
    case 54:
        return "6";
    case 55:
        return "7";
    case 56:
        return "8";
    case 57:
        return "9";
    case 97:
        return "a";
    case 98:
        return "b";
    case 99:
        return "c";
    case 100:
        return "d";
    case 101:
        return "e";
    case 102:
        return "f";
    case 8:
        backspace = true;
        return "";

    default:
        return "";
    }
}

uint16_t fromHexToUint(const std::string &hexValue) {
    uint16_t value = 0;
    std::stringstream ss;
    ss << std::hex << hexValue;
    ss >> value;
    return value;
}

} // namespace detail

AdiTofDemoView::AdiTofDemoView(std::shared_ptr<AdiTofDemoController> &ctrl,
                               const std::string &name)
    : m_ctrl(ctrl), m_viewName(name), m_depthFrameAvailable(false),
      m_irFrameAvailable(false), m_stopWorkersFlag(false), m_center(true),
      m_waitKeyBarrier(0), m_distanceVal(0), m_crtSmallSignalState(false),
      m_crtIRGamma(false) {
    // cv::setNumThreads(2);
    m_depthImageWorker =
        std::thread(std::bind(&AdiTofDemoView::_displayDepthImage, this));
    m_irImageWorker =
        std::thread(std::bind(&AdiTofDemoView::_displayIrImage, this));
}
AdiTofDemoView::~AdiTofDemoView() {
    std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
    m_stopWorkersFlag = true;
    lock.unlock();
    m_frameCapturedCv.notify_all();
    m_depthImageWorker.join();
    m_irImageWorker.join();
}

void AdiTofDemoView::render() {

    bool nearModeChecked = true;
    bool mediumModeChecked = false;
    bool farModeChecked = false;
    int modeCurrentValue = 4; // 4 = near(default); 2 = medium; 1 = far

    bool lowLevelChecked = false;
    bool mediumLevelChecked = false;
    bool highLevelChecked = false;
    int levelCurrentValue = 2; // 4 = low; 2 = mediu(default); 1 = high

    bool livePlayChecked = true;
    bool playbackChecked = false;
    int playCurrentValue = 2;

    bool separatedViewChecked = true;
    bool blendedViewChecked = false;
    int viewCurrentValue = 2;

    std::string typeOfDevice = DEVICETYPE;
    std::string typeUSBDevice = "USB";
    std::string typeLocalDevice = "Local";
    std::string typeEthDevice = "ETH";

    bool USBModeChecked = false;
    bool localModeChecked = false;
    bool ethModeChecked = false;
    int connectionCurrentValue = 4;

    bool revBChecked = false;
    bool revCChecked = true;
    int revCurrentValue = 1; // 2 = RevB; 1 = RevC(default)
    std::string revisions[2] = {"RevB", "RevC"};

    if (typeOfDevice.find(typeUSBDevice) != std::string::npos) {
        USBModeChecked = true;
    } else {
        if (typeOfDevice.find(typeLocalDevice) != std::string::npos) {
            localModeChecked = true;
        } else {
            ethModeChecked = true;
        }
    }

    int pulseCount = 2000;

    unsigned int normalColor = 0x000032;
    unsigned int selectedColor = 0xffffff;
    unsigned int errorColor = 0xff0000;

    int thresholdClicked = 0;
    int smallSignalThreshold = 50;
    float IRGamma = 1.0f;

    std::string ipvalue = "";
    bool ipFieldSelected = false;
    unsigned int ipColor = normalColor;
    bool ipConnectionEnabled = false;

    std::string address = "0x";
    std::string valueSTh = std::to_string(smallSignalThreshold);
    std::string valueG = std::to_string(IRGamma);
    std::string fileName = "";
    std::string status = "";

    unsigned int addressColor = normalColor;
    unsigned int valueColorSTh = normalColor;
    unsigned int valueColorG = normalColor;
    unsigned int fieldColor = normalColor;

    bool addressFieldSelected = false;
    bool valueFieldSelectedSTh = false;
    bool valueFieldSelectedG = false;
    bool fileNameFieldSelected = false;

    const cv::String windows[] = {m_viewName, "Depth Image", "IR Image",
                                  "Blended Image"};
    cvui::init(windows, 1);

    int frameCount = 0;
    int displayFps = 0;
    auto startTime = std::chrono::system_clock::now();

    bool captureEnabled = false;
    bool captureBlendedEnabled = false;
    bool recordEnabled = false;
    bool playbackEnabled = false;

    int numberOfFrames = 0;

    std::string modes[3] = {"near", "medium", "far"};

    char afe_temp_str[32] = "AFE TEMP:";
    char laser_temp_str[32] = "LASER TEMP:";
    int temp_cnt = 0;

    while (true) {

        // Fill the frame with a nice color
        cv::Mat frame = cv::Mat(620, 400, CV_8UC3);

        frame = cv::Scalar(49, 52, 49);

        cvui::context(m_viewName);

        // Exit button
        if (cvui::button(frame, 375, 0, 25, 25, "X")) {
            break;
        }

        bool checkboxChanged = false;

        // Mode checkbox group
        int btnGroupMode =
            nearModeChecked << 2 | mediumModeChecked << 1 | farModeChecked;
        if (modeCurrentValue != btnGroupMode) {
            int xorValue = modeCurrentValue ^ btnGroupMode;
            modeCurrentValue = xorValue;
            nearModeChecked = xorValue & (1 << 2);
            mediumModeChecked = xorValue & (1 << 1);
            farModeChecked = xorValue & 1;
            checkboxChanged = true;
        }

        // Level checkbox group
        int btnGroupLevel =
            lowLevelChecked << 2 | mediumLevelChecked << 1 | highLevelChecked;
        if (levelCurrentValue != btnGroupLevel) {
            int xorValue = levelCurrentValue ^ btnGroupLevel;
            levelCurrentValue = xorValue;
            lowLevelChecked = xorValue & (1 << 2);
            mediumLevelChecked = xorValue & (1 << 1);
            highLevelChecked = xorValue & 1;
            checkboxChanged = true;
        }

        cvui::beginColumn(frame, 50, 520);
        cvui::space(10);
        cvui::text("Revision: ", 0.6);
        cvui::space(10);
        cvui::beginRow(frame, 50, 555);
        cvui::checkbox("RevB", &revBChecked);
        cvui::space(10);
        cvui::checkbox("RevC", &revCChecked);
        cvui::endRow();
        cvui::endColumn();

        // Rev checkbox group
        int btnGroupRev = revBChecked << 1 | revCChecked;
        if (revCurrentValue != btnGroupRev) {
            int xorValue = revCurrentValue ^ btnGroupRev;
            revCurrentValue = xorValue;
            revBChecked = xorValue & (1 << 1);
            revCChecked = xorValue & 1;

            if (revBChecked) {
                m_ctrl->setCameraRevision(revisions[0]);
            } else {
                m_ctrl->setCameraRevision(revisions[1]);
            }
        }

        // play button group
        int btnGroupPlay = livePlayChecked << 1 | playbackChecked;
        if (playCurrentValue != btnGroupPlay) {
            int xorValue = playCurrentValue ^ btnGroupPlay;
            playCurrentValue = xorValue;
            livePlayChecked = xorValue & (1 << 1);
            playbackChecked = xorValue & 1;
        }

        // Creating a group of buttons where only one can be active at a time
        int btnGroupView = separatedViewChecked << 1 | blendedViewChecked;
        if (viewCurrentValue != btnGroupView) {
            int xorValue = viewCurrentValue ^ btnGroupView;
            viewCurrentValue = xorValue;
            separatedViewChecked = xorValue & (1 << 1);
            blendedViewChecked = xorValue & 1;
        }

        // TODO: set camera mode here
        if (checkboxChanged) {
            int selectedMode =
                (2 - static_cast<int>(std::log2(modeCurrentValue)));
            m_ctrl->setMode(modes[selectedMode]);
            status = "Mode set: " + modes[selectedMode];
            m_crtSmallSignalState = false;
        }

        // Connection mode checkbox group
        bool connectionCheckboxChanged = false;
        if (USBModeChecked == true) {
            int btnGroupConnection =
                USBModeChecked << 2 | localModeChecked << 1 | ethModeChecked;
            if (connectionCurrentValue != btnGroupConnection) {
                int xorValue = connectionCurrentValue ^ btnGroupConnection;
                connectionCurrentValue = xorValue;
                USBModeChecked = xorValue & (1 << 2);
                localModeChecked = xorValue & (1 << 1);
                ethModeChecked = xorValue & 1;
                connectionCheckboxChanged = true;
            }
        }

        // IP connection GUI //470, 50
        if (ethModeChecked) {
            cvui::rect(frame, 50, 80, 170, 30, ipColor);
            cvui::text(frame, 60, 90, ipvalue);
            int ipField = cvui::iarea(50, 80, 170, 30);
            if (ipField == cvui::CLICK) {
                ipColor = selectedColor;
                ipFieldSelected = true;
            } else if (cvui::mouse(cvui::CLICK) && ipField != cvui::CLICK) {
                ipColor = normalColor;
                ipFieldSelected = false;
            }
            if (cvui::button(frame, 230, 80, 90, 30, "Connect")) {
                ipConnectionEnabled = !ipConnectionEnabled;
            }
        }

        // set connection mode
        if (connectionCheckboxChanged || ipConnectionEnabled) {
            if (ipConnectionEnabled) {
                if (!ipvalue.empty()) {
                    bool connectionResult =
                        m_ctrl->setEthernetConnection(ipvalue);
                    if (connectionResult == true) {
                        int selectedMode =
                            (2 - static_cast<int>(std::log2(modeCurrentValue)));
                        m_ctrl->setMode(modes[selectedMode]);
                    }
                }
                ipConnectionEnabled = !ipConnectionEnabled;
            }

            if (USBModeChecked) {
                bool connectionResult = m_ctrl->setRegularConnection();
                if (connectionResult == true) {
                    int selectedMode =
                        (2 - static_cast<int>(std::log2(modeCurrentValue)));
                    m_ctrl->setMode(modes[selectedMode]);
                }
            }

            if (localModeChecked) {
                bool connectionResult = m_ctrl->setRegularConnection();
                if (connectionResult == true) {
                    int selectedMode =
                        (2 - static_cast<int>(std::log2(modeCurrentValue)));
                    m_ctrl->setMode(modes[selectedMode]);
                }
            }
        }

        cvui::beginColumn(frame, 50, 105);
        cvui::space(10);
        cvui::text("Mode: ", 0.6);
        cvui::space(10);
        cvui::beginRow(frame, 50, 140);
        cvui::checkbox("Near", &nearModeChecked);
        cvui::space(10);
        cvui::checkbox("Medium", &mediumModeChecked);
        cvui::space(10);
        cvui::checkbox("Far", &farModeChecked);
        cvui::endRow();
        cvui::endColumn();

        cvui::text(frame, 50, 160, "Video: ", 0.6);

        if (cvui::button(
                frame, 50, 185, 90, 30,
                (captureEnabled || captureBlendedEnabled ? "Stop" : "Play"))) {
            if (livePlayChecked) {
                if (separatedViewChecked) {
                    if (m_ctrl->hasCamera()) {
                        captureEnabled = !captureEnabled;
                        if (captureEnabled) {
                            m_ctrl->startCapture();
                            m_ctrl->requestFrame();
                            status = "Playing from device!";
                        } else {
                            cv::destroyWindow(windows[1]);
                            cv::destroyWindow(windows[2]);
                            m_ctrl->stopCapture();
                            status = "";
                        }
                    } else {
                        status = "No cameras connected!";
                    }
                } else {
                    if (m_ctrl->hasCamera()) {
                        captureBlendedEnabled = !captureBlendedEnabled;
                        if (captureBlendedEnabled) {
                            m_ctrl->startCapture();
                            m_ctrl->requestFrame();
                            status = "Playing from device!";
                        } else {
                            cv::destroyWindow(windows[3]);
                            m_ctrl->stopCapture();
                            status = "";
                        }
                    } else {
                        status = "No cameras connected!";
                    }
                }
            } else {
                if (fileName.empty()) {
                    fieldColor = errorColor;
                    status = "Enter a valid file name!";
                } else {
                    playbackEnabled = !playbackEnabled;
                    static std::string oldStatus = "";
                    captureEnabled = !captureEnabled;
                    if (playbackEnabled) {
                        oldStatus = status;
                        status = "Playing from file: " + fileName;
                        numberOfFrames =
                            m_ctrl->startPlayback(fileName, displayFps);
                    } else {
                        status = oldStatus;
                        m_ctrl->stopPlayback();
                        cv::destroyWindow(windows[1]);
                        cv::destroyWindow(windows[2]);
                    }
                }
            }
        }
        if (cvui::button(frame, 150, 185, 90, 30, "Record")) {
            if (fileName.empty()) {
                status = "Enter a file name!";
            } else if (captureEnabled || captureBlendedEnabled) {
                recordEnabled = !recordEnabled;
                static std::string oldStatus = "";
                if (recordEnabled) {
                    oldStatus = status;
                    status = "Recording into " + fileName;
                    aditof::FrameDetails frameDetails;
                    m_capturedFrame->getDetails(frameDetails);
                    m_ctrl->startRecording(
                        fileName, frameDetails,
                        static_cast<unsigned int>(displayFps));
                } else {
                    m_ctrl->stopRecording();
                    status = oldStatus;
                }
            } else {
                status = "Start live playing before recording!";
            }
        }

        cvui::rect(frame, 50, 220, 190, 30, fieldColor);
        cvui::text(frame, 60, 230, fileName);

        cvui::text(frame, 50, 255, status);
        cvui::beginRow(frame, 50, 273);
        cvui::checkbox("Live", &livePlayChecked);
        cvui::space(10);
        cvui::checkbox("Playback", &playbackChecked);
        cvui::endRow();

        if (!(captureEnabled || captureBlendedEnabled || playbackChecked)) {
            cvui::beginColumn(frame, 50, 300);
            cvui::text("View depth and IR: ", 0.6);
            cvui::space(10);
            cvui::beginRow(frame, 50, 325);
            cvui::checkbox("Separated", &separatedViewChecked);
            cvui::space(10);
            cvui::checkbox("Blended", &blendedViewChecked);
            cvui::endRow();
            cvui::endColumn();
        } else if (captureBlendedEnabled) {
            cvui::beginColumn(frame, 50, 300);
            cvui::text("Blending factor:", 0.6);
            cvui::space(5);
            cvui::trackbar(200, &m_blendValue, (double)0.0, (double)1.0);
            cvui::space(5);
            cvui::endColumn();
        } else {
            cvui::beginColumn(frame, 50, 300);
            cvui::text("Showing depth and IR", 0.6);
            ;
            cvui::endColumn();
        }

        // Connection mode GUI
        cvui::beginColumn(frame, 50, 20);
        cvui::space(10);
        cvui::text("Connection Mode: ", 0.6);
        cvui::space(10);
        cvui::beginRow(frame, 50, 60);
        if (typeOfDevice.find(typeUSBDevice) != std::string::npos) {
            cvui::checkbox("USB", &USBModeChecked);
            cvui::space(10);
        }
        if (typeOfDevice.find(typeLocalDevice) != std::string::npos) {
            cvui::checkbox("Local", &localModeChecked);
            cvui::space(10);
        }
        if (typeOfDevice.find(typeEthDevice) != std::string::npos) {
            cvui::checkbox("Ethernet", &ethModeChecked);
        }
        cvui::endRow();
        cvui::endColumn();

        static int currentFrame = 0;
        if (playbackEnabled) {
            int x = currentFrame++;
            cvui::trackbar(frame, 300, 160, 120, &x, 0, numberOfFrames + 1, 1);
        } else {
            currentFrame = 0;
        }

        int fileNameField = cvui::iarea(50, 220, 190, 30);

        if (fileNameField == cvui::CLICK) {
            fieldColor = selectedColor;
            fileNameFieldSelected = true;
        } else if (cvui::mouse(cvui::CLICK) && fileNameField != cvui::CLICK) {
            fieldColor = normalColor;
            fileNameFieldSelected = false;
        }

#ifdef SHOW_ADVANCED_GUI
        cvui::beginColumn(frame, 300, 20);
        cvui::space(10);
        cvui::text("Level: ", 0.6);
        cvui::space(10);
        cvui::checkbox("Low", &lowLevelChecked);
        cvui::space(10);
        cvui::checkbox("Medium", &mediumLevelChecked);
        cvui::space(10);
        cvui::checkbox("High", &highLevelChecked);
        cvui::endColumn();

        cvui::beginColumn(frame, 50, 140);
        cvui::space(10);
        cvui::text("Pulse Count: ", 0.6);
        cvui::space(10);
        cvui::trackbar(300, &pulseCount, 0, 2500, 1);
        cvui::endColumn();
        // TODO: set pulse count here
        // m_ctrl->setPulseCount(pulseCount);

        cvui::beginColumn(frame, 50, 240);
        cvui::space(20);
        cvui::text("Registers:", 0.6);
        cvui::space(20);
        cvui::text("Address:");
        cvui::space(25);
        cvui::text("Value:");
        cvui::endColumn();

        // Draw input rectangles and text displays for address / value
        cvui::text(frame, 125, 295, address);
        cvui::text(frame, 125, 330, value);

        cvui::rect(frame, 120, 287, 90, 30, addressColor);
        cvui::rect(frame, 120, 322, 90, 30, valueColor);

        int statusAddress = cvui::iarea(120, 287, 90, 30);
        int statusValue = cvui::iarea(120, 322, 90, 30);

        if (statusAddress == cvui::CLICK) {
            addressFieldSelected = true;
            valueFieldSelected = false;
            addressColor = selectedColor;
            valueColor = normalColor;
        }
        if (statusValue == cvui::CLICK) {
            valueFieldSelected = true;
            addressFieldSelected = false;
            valueColor = selectedColor;
            addressColor = normalColor;
        }

        if (cvui::mouse(cvui::CLICK) && statusAddress != cvui::CLICK &&
            statusValue != cvui::CLICK) {
            valueFieldSelected = false;
            addressFieldSelected = false;
            valueColor = normalColor;
            addressColor = normalColor;
        }

        if (cvui::button(frame, 225, 287, 90, 30, "Read")) {
            if (address.size() <= 2) {
                addressColor = errorColor;
            } else {
                uint16_t addr = detail::fromHexToUint(address);
                uint16_t data = 0;
                // TODO: read afe reg here
                // m_ctrl->readAFEregister(&addr, &data);
                // value = "0x" + std::to_string(data);
            }
            valueFieldSelected = false;
            addressFieldSelected = false;
        }
        if (cvui::button(frame, 225, 322, 90, 30, "Write")) {
            if (address.size() <= 2) {
                addressColor = errorColor;
            }
            if (value.size() <= 2) {
                valueColor = errorColor;
            }
            if (address.size() > 2 && value.size() > 2) {
                // TODO: write afe reg here!
                // uint16 addr = detail::fromHexToUint(address);
                // uint16 data = detail::fromHexToUint(value);
                // m_ctrl->writeAFEregister(&addr, &data);
            }
            valueFieldSelected = false;
            addressFieldSelected = false;
        }
#endif
        if (displayFps && (captureEnabled || captureBlendedEnabled)) {
            cvui::text(frame, 350, 580, "FPS:" + std::to_string(displayFps));
        }

        if (captureEnabled || captureBlendedEnabled) {
            if (temp_cnt++ == 64) {
                std::pair<float, float> temp = m_ctrl->getTemperature();
                temp_cnt = 0;
                sprintf(afe_temp_str, "AFE TEMP: %.1f", temp.first);
                sprintf(laser_temp_str, "LASER TEMP: %.1f", temp.second);
            }
            cvui::text(frame, 20, 580, afe_temp_str);
            cvui::text(frame, 180, 580, laser_temp_str);
        }

        if ((captureEnabled || captureBlendedEnabled) && !playbackEnabled) {
            frameCount++;
            auto endTime = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed = endTime - startTime;
            if (elapsed.count() >= 2) {
                displayFps = frameCount / elapsed.count();
                frameCount = 0;
                startTime = endTime;
            }
        }

        static bool lastSmallSignalState = false;

        cvui::beginColumn(frame, 50, 350);
        cvui::space(10);
        cvui::text("Settings: ", 0.6);
        cvui::space(10);
        cvui::checkbox("Center Point", &m_center);
        cvui::space(10);
        cvui::checkbox("Small signal removal (0 - 16383)",
                       &m_crtSmallSignalState);
        cvui::space(10);
        cvui::endColumn();

        cvui::rect(frame, 50, 430, 100, 30, valueColorSTh);
        cvui::text(frame, 60, 440, valueSTh);
        int thresholdClicked = cvui::iarea(50, 430, 100, 30);

        // Check if small signal toggle button has changed
        if (m_crtSmallSignalState != lastSmallSignalState) {
            aditof::Status ret =
                m_ctrl->enableNoiseReduction(m_crtSmallSignalState);
            if (ret == aditof::Status::OK) {
                m_ctrl->setNoiseReductionThreshold(
                    static_cast<uint16_t>(smallSignalThreshold));
            } else if (ret == aditof::Status::GENERIC_ERROR) {
                status = "No cameras connected!";
                m_crtSmallSignalState = false;
            } else if (ret == aditof::Status::UNAVAILABLE) {
                status = "Far mode does not support noise reduction";
                m_crtSmallSignalState = false;
            }
            lastSmallSignalState = m_crtSmallSignalState;
        }

        if (cvui::button(frame, 160, 430, 90, 30, "Write")) {
            aditof::Status noiseReductionStatus =
                m_ctrl->setNoiseReductionThreshold(
                    static_cast<uint16_t>(smallSignalThreshold));

            if (noiseReductionStatus == aditof::Status::GENERIC_ERROR) {
                status = "No cameras connected!";
            } else if (noiseReductionStatus == aditof::Status::UNAVAILABLE) {
                status = "Far mode does not support noise reduction";
            }
        }

        static bool lastIRGamma = false;

        cvui::beginColumn(frame, 50, 460);
        cvui::space(10);
        cvui::checkbox("IR gamma correction", &m_crtIRGamma);
        cvui::space(10);
        cvui::endColumn();

        cvui::rect(frame, 50, 490, 100, 30, valueColorG);
        cvui::text(frame, 60, 500, valueG);
        int gammaClicked = cvui::iarea(50, 490, 100, 30);

        if (m_crtIRGamma != lastIRGamma) {
            if (m_crtIRGamma) {
                aditof::Status ret = m_ctrl->setIrGammaCorrection(IRGamma);
                if (ret == aditof::Status::GENERIC_ERROR) {
                    status = "No cameras connected!";
                    m_crtIRGamma = false;
                }
            } else {
                aditof::Status ret = m_ctrl->setIrGammaCorrection(1.0f);
                if (ret == aditof::Status::GENERIC_ERROR) {
                    status = "No cameras connected!";
                    m_crtIRGamma = false;
                }
            }
            lastIRGamma = m_crtIRGamma;
        }

        if (cvui::button(frame, 160, 490, 90, 30, "Write")) {
            if (m_crtIRGamma) {
                aditof::Status ret = m_ctrl->setIrGammaCorrection(IRGamma);

                if (ret == aditof::Status::GENERIC_ERROR) {
                    status = "No cameras connected!";
                }
            }
        }

        if (gammaClicked == cvui::CLICK) {
            valueColorG = selectedColor;
            valueFieldSelectedG = true;
        } else if (cvui::mouse(cvui::CLICK) && gammaClicked != cvui::CLICK) {
            valueColorG = normalColor;
            valueFieldSelectedG = false;
        }

        if (thresholdClicked == cvui::CLICK) {
            valueColorSTh = selectedColor;
            valueFieldSelectedSTh = true;
        } else if (cvui::mouse(cvui::CLICK) &&
                   thresholdClicked != cvui::CLICK) {
            valueColorSTh = normalColor;
            valueFieldSelectedSTh = false;
        }

        cvui::imshow(m_viewName, frame);

        if (captureEnabled) {
            if (playbackEnabled) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1000 / displayFps));
            }

            if (m_ctrl->playbackFinished() && playbackEnabled) {
                captureEnabled = false;
                playbackEnabled = false;
                status = "Finished playing from: " + fileName;
                m_ctrl->stopPlayback();
                cv::destroyWindow(windows[1]);
                cv::destroyWindow(windows[2]);
            } else if (!captureBlendedEnabled) {
                m_capturedFrame = m_ctrl->getFrame();

                aditof::FrameDetails frameDetails;
                m_capturedFrame->getDetails(frameDetails);

                std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
                m_depthFrameAvailable = true;
                m_irFrameAvailable = true;
                lock.unlock();
                m_frameCapturedCv.notify_all();
                m_ctrl->requestFrame();
            }
        }

        if (captureBlendedEnabled) {
            m_capturedFrame = m_ctrl->getFrame();

            aditof::FrameDetails frameDetails;
            m_capturedFrame->getDetails(frameDetails);
            m_ctrl->requestFrame();
        }

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);
        if (captureEnabled && !captureBlendedEnabled) {
            m_barrierCv.wait(imshow_lock,
                             [&]() { return m_waitKeyBarrier == 2; });
            m_waitKeyBarrier = 0;

            cvui::imshow("Depth Image", m_depthImage);
            cvui::imshow("IR Image", m_irImage);
            m_depthImage.release();
            m_irImage.release();
        }

        if (captureBlendedEnabled) {
            _displayBlendedImage();
            cvui::imshow("Blended Image", m_blendedImage);
            m_blendedImage.release();
        }

        int key = cv::waitKey(10);

        bool backspace = false;
        std::string pressedValidKey = detail::getKeyPressed(key, backspace);

        if (valueFieldSelectedSTh) {
            if (key >= 47 && key <= 57) {
                valueSTh += pressedValidKey;
                int currentValue = stoi(valueSTh);
                if (currentValue > ((1 << 14) - 1)) {
                    valueSTh = valueSTh.substr(0, valueSTh.size() - 1);
                } else {
                    smallSignalThreshold = currentValue;
                }
            } else if (backspace) {
                valueSTh = valueSTh.substr(0, valueSTh.size() - 1);
                if (valueSTh.compare("") != 0) {
                    smallSignalThreshold = stoi(valueSTh);
                }
            }
        }

        if (valueFieldSelectedG) {
            if ((key >= 48 && key <= 57) || (key == 46)) {
                if ((valueG.size() < 11)) {
                    try {
                        valueG += pressedValidKey;
                        IRGamma = stof(valueG);
                    } catch (const std::invalid_argument &ia) {
                        status = "Input for IRGamma must be float!";
                        valueG = valueG.substr(0, valueG.size() - 1);
                    }
                }
            } else if (backspace) {
                valueG = valueG.substr(0, valueG.size() - 1);
                if (valueG.compare("") != 0) {
                    IRGamma = stof(valueG);
                }
            }
        }

        if (addressFieldSelected) {
            if (!backspace && address.size() < 6) {
                address += pressedValidKey;
            } else if (backspace && address.size() > 2) {
                address = address.substr(0, address.size() - 1);
            }
        }

        if (fileNameFieldSelected) {
            if (!backspace) {
                fileName += pressedValidKey;
            } else if (fileName.size() > 0) {
                fileName = fileName.substr(0, fileName.size() - 1);
            }
        }

        if (ipFieldSelected) {
            if (!backspace) {
                ipvalue += pressedValidKey;
            } else if (ipvalue.size() > 0) {
                ipvalue = ipvalue.substr(0, ipvalue.size() - 1);
            }
        }
        // Check if ESC key was pressed
        if (key == 27) {
            break;
        }
    }
    m_ctrl->stopCapture();
}

void AdiTofDemoView::_displayDepthImage() {
    while (!m_stopWorkersFlag) {
        std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
        m_frameCapturedCv.wait(
            lock, [&]() { return m_depthFrameAvailable || m_stopWorkersFlag; });

        if (m_stopWorkersFlag) {
            break;
        }

        m_depthFrameAvailable = false;

        std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;
        lock.unlock(); // Lock is no longer needed

        uint16_t *data;
        localFrame->getData(aditof::FrameDataType::DEPTH, &data);

        aditof::FrameDetails frameDetails;
        localFrame->getDetails(frameDetails);

        int frameHeight = static_cast<int>(frameDetails.height) / 2;
        int frameWidth = static_cast<int>(frameDetails.width);

        m_depthImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, data);
        cv::Mat m_distanceImage =
            cv::Mat(frameHeight, frameWidth, CV_16UC1, data);
        cv::Point2d pointxy(320, 240);
        m_distanceVal = static_cast<int>(
            m_distanceVal * 0.7 + m_distanceImage.at<ushort>(pointxy) * 0.3);
        char text[20];
        sprintf(text, "%dmm", m_distanceVal);
        m_depthImage.convertTo(
            m_depthImage, CV_8U,
            (255.0 / (m_ctrl->getRangeMax() - m_ctrl->getRangeMin())),
            (-(255.0 / (m_ctrl->getRangeMax() - m_ctrl->getRangeMin())) *
             m_ctrl->getRangeMin()));
        applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_RAINBOW);
        flip(m_depthImage, m_depthImage, 1);
        int color;
        if (m_distanceVal > 2500)
            color = 0;
        else
            color = 4096;

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);
        if (m_center) {
#ifndef OPENCV2
            cv::drawMarker(m_depthImage, pointxy,
                           cv::Scalar(color, color, color), cv::MARKER_CROSS);
#else
            cv::line(m_depthImage, cv::Point(pointxy.x - 10, pointxy.y),
                     cv::Point(pointxy.x + 10, pointxy.y),
                     cv::Scalar(color, color, color));
            cv::line(m_depthImage, cv::Point(pointxy.x, pointxy.y - 10),
                     cv::Point(pointxy.x, pointxy.y + 10),
                     cv::Scalar(color, color, color));
#endif
            cv::circle(m_depthImage, pointxy, 8,
                       cv::Scalar(color, color, color));
            cv::putText(m_depthImage, text, pointxy + cv::Point2d(10, 20),
                        cv::FONT_HERSHEY_DUPLEX, 2,
                        cv::Scalar(color, color, color));
        }
        m_waitKeyBarrier += 1;
        if (m_waitKeyBarrier == 2) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }
}

void AdiTofDemoView::_displayIrImage() {
    while (!m_stopWorkersFlag) {
        std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
        m_frameCapturedCv.wait(
            lock, [&]() { return m_irFrameAvailable || m_stopWorkersFlag; });

        if (m_stopWorkersFlag) {
            break;
        }

        m_irFrameAvailable = false;

        std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;
        lock.unlock(); // Lock is no longer needed

        uint16_t *irData;
        localFrame->getData(aditof::FrameDataType::IR, &irData);

        aditof::FrameDetails frameDetails;
        localFrame->getDetails(frameDetails);

        int frameHeight = static_cast<int>(frameDetails.height) / 2;
        int frameWidth = static_cast<int>(frameDetails.width);
        int max_value_of_IR_pixel = (1 << m_ctrl->getbitCount()) - 1;

        m_irImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, irData);
        m_irImage.convertTo(m_irImage, CV_8U, 255.0 / max_value_of_IR_pixel);
        flip(m_irImage, m_irImage, 1);

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);
        m_waitKeyBarrier += 1;
        if (m_waitKeyBarrier == 2) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }
}

void AdiTofDemoView::_displayBlendedImage() {
    std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;

    uint16_t *irData;
    localFrame->getData(aditof::FrameDataType::IR, &irData);

    uint16_t *data;
    localFrame->getData(aditof::FrameDataType::DEPTH, &data);

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);

    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);
    int max_value_of_IR_pixel = (1 << m_ctrl->getbitCount()) - 1;

    m_irImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, irData);
    m_irImage.convertTo(m_irImage, CV_8U, 255.0 / max_value_of_IR_pixel);
    flip(m_irImage, m_irImage, 1);
    cv::cvtColor(m_irImage, m_irImage, cv::COLOR_GRAY2RGB);

    m_depthImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, data);
    m_depthImage.convertTo(
        m_depthImage, CV_8U,
        (255.0 / (m_ctrl->getRangeMax() - m_ctrl->getRangeMin())),
        (-(255.0 / (m_ctrl->getRangeMax() - m_ctrl->getRangeMin())) *
         m_ctrl->getRangeMin()));
    flip(m_depthImage, m_depthImage, 1);
    applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_RAINBOW);

    cv::addWeighted(m_depthImage, m_blendValue, m_irImage, 1.0F - m_blendValue,
                    0, m_blendedImage);
}
