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
#include "aditofdemorecorder.h"

#include <functional>
#include <string.h>

AditofDemoRecorder::AditofDemoRecorder()
    : m_frameDetails{0, 0, 0, 0, 0, 0, ""}, m_recordTreadStop(true),
      m_playbackThreadStop(true), m_shouldReadNewFrame(true),
      m_playBackEofReached(false), m_numberOfFrames(0) {}

AditofDemoRecorder::~AditofDemoRecorder() {
    if (m_recordFile.is_open()) {
        stopRecording();
    }
    if (m_playbackFile.is_open()) {
        stopPlayback();
    }
}

void AditofDemoRecorder::startRecording(
    const std::string &fileName, const aditof::FrameDetails &frameDetails,
    const aditof::CameraDetails &cameraDetails, unsigned int fps) {
    m_recordFile.open(fileName, std::ios::binary);

    m_frameDetails.height = static_cast<int>(frameDetails.height);
    m_frameDetails.width = static_cast<int>(frameDetails.width);
    m_frameDetails.rgbHeight = static_cast<int>(frameDetails.rgbHeight);
    m_frameDetails.rgbWidth = static_cast<int>(frameDetails.rgbWidth);
    m_frameDetails.type = static_cast<std::string>(frameDetails.type);

    if (frameDetails.type.find("rgb") != std::string::npos) {
        m_rgbEnabled = 1;
    } else {
        m_rgbEnabled = 0;
    }
    /* Header version */
    unsigned char version = FILE_HEADER_VERSION;
    m_recordFile.write(reinterpret_cast<const char *>(&version),
                       sizeof(unsigned char));
    /* Frame Height, Width, type and FPS */
    m_recordFile.write(reinterpret_cast<const char *>(&m_frameDetails.height),
                       sizeof(unsigned int));
    m_recordFile.write(reinterpret_cast<const char *>(&m_frameDetails.width),
                       sizeof(unsigned int));
    m_recordFile.write(
        reinterpret_cast<const char *>(&m_frameDetails.rgbHeight),
        sizeof(unsigned int));
    m_recordFile.write(reinterpret_cast<const char *>(&m_frameDetails.rgbWidth),
                       sizeof(unsigned int));
    m_recordFile.write(reinterpret_cast<const char *>(&m_rgbEnabled),
                       sizeof(uint8_t));
    m_recordFile.write(reinterpret_cast<const char *>(&fps),
                       sizeof(unsigned int));
    /* Depth gain and offset */
    m_recordFile.write(reinterpret_cast<const char *>(
                           &cameraDetails.depthParameters.depthGain),
                       sizeof(float));
    m_recordFile.write(reinterpret_cast<const char *>(
                           &cameraDetails.depthParameters.depthOffset),
                       sizeof(float));
    /* bit count */
    m_recordFile.write(reinterpret_cast<const char *>(&cameraDetails.bitCount),
                       sizeof(int));
    /* min and max depth */
    m_recordFile.write(
        reinterpret_cast<const char *>(&cameraDetails.depthParameters.minDepth),
        sizeof(int));
    m_recordFile.write(
        reinterpret_cast<const char *>(&cameraDetails.depthParameters.maxDepth),
        sizeof(int));
    /* intrinsics pixel height and width */
    m_recordFile.write(
        reinterpret_cast<const char *>(&cameraDetails.intrinsics.pixelHeight),
        sizeof(float));
    m_recordFile.write(
        reinterpret_cast<const char *>(&cameraDetails.intrinsics.pixelWidth),
        sizeof(float));
    /* intrinsics fx, fy, cx and cy */
    float fx = cameraDetails.intrinsics.cameraMatrix.at(0);
    m_recordFile.write(reinterpret_cast<const char *>(&fx), sizeof(float));
    float fy = cameraDetails.intrinsics.cameraMatrix.at(4);
    m_recordFile.write(reinterpret_cast<const char *>(&fy), sizeof(float));
    float cx = cameraDetails.intrinsics.cameraMatrix.at(2);
    m_recordFile.write(reinterpret_cast<const char *>(&cx), sizeof(float));
    float cy = cameraDetails.intrinsics.cameraMatrix.at(5);
    m_recordFile.write(reinterpret_cast<const char *>(&cy), sizeof(float));
    /* intrinsics distorsion coefs k1, k2, p1, p2, k3 */
    float k1 = cameraDetails.intrinsics.distCoeffs.at(0);
    m_recordFile.write(reinterpret_cast<const char *>(&k1), sizeof(float));
    float k2 = cameraDetails.intrinsics.distCoeffs.at(1);
    m_recordFile.write(reinterpret_cast<const char *>(&k2), sizeof(float));
    float p1 = cameraDetails.intrinsics.distCoeffs.at(2);
    m_recordFile.write(reinterpret_cast<const char *>(&p1), sizeof(float));
    float p2 = cameraDetails.intrinsics.distCoeffs.at(3);
    m_recordFile.write(reinterpret_cast<const char *>(&p2), sizeof(float));
    float k3 = cameraDetails.intrinsics.distCoeffs.at(4);
    m_recordFile.write(reinterpret_cast<const char *>(&k3), sizeof(float));
    m_recordTreadStop = false;
    m_recordThread =
        std::thread(std::bind(&AditofDemoRecorder::recordThread, this));
}

void AditofDemoRecorder::stopRecording() {
    m_recordTreadStop = true;
    if (m_recordThread.joinable()) {
        m_recordThread.join();
    }
    m_recordFile.close();
}

int AditofDemoRecorder::startPlayback(const std::string &fileName, int &fps) {
    m_playbackFile.open(fileName, std::ios::binary);
    if (!m_playbackFile.is_open()) {
        return 0;
    }
    m_playbackFile.seekg(0, std::ios_base::end);
    int fileSize = (int)m_playbackFile.tellg();
    m_playbackFile.seekg(0, std::ios_base::beg);

    /* Header version */
    unsigned char version;
    m_playbackFile.read(reinterpret_cast<char *>(&version),
                        sizeof(unsigned char));
    if (version > FILE_HEADER_VERSION) {
        return 0;
    }
    /* Frame Height, Width and FPS */
    int frameHeight;
    m_playbackFile.read(reinterpret_cast<char *>(&frameHeight),
                        sizeof(unsigned int));
    int frameWidth;
    m_playbackFile.read(reinterpret_cast<char *>(&frameWidth),
                        sizeof(unsigned int));
    int rgbFrameHeight;
    m_playbackFile.read(reinterpret_cast<char *>(&rgbFrameHeight),
                        sizeof(unsigned int));
    int rgbFrameWidth;
    m_playbackFile.read(reinterpret_cast<char *>(&rgbFrameWidth),
                        sizeof(unsigned int));
    uint8_t rgbEnabled;
    m_playbackFile.read(reinterpret_cast<char *>(&rgbEnabled), sizeof(char));
    m_playbackFile.read(reinterpret_cast<char *>(&fps), sizeof(unsigned int));
    /* Depth gain and offset */
    m_playbackFile.read(
        reinterpret_cast<char *>(&m_recorderDetails.depthParameters.depthGain),
        sizeof(float));
    m_playbackFile.read(reinterpret_cast<char *>(
                            &m_recorderDetails.depthParameters.depthOffset),
                        sizeof(float));
    /* bit count */
    m_playbackFile.read(reinterpret_cast<char *>(&m_recorderDetails.bitCount),
                        sizeof(int));
    /* min and max depth */
    m_playbackFile.read(
        reinterpret_cast<char *>(&m_recorderDetails.depthParameters.minDepth),
        sizeof(int));
    m_playbackFile.read(
        reinterpret_cast<char *>(&m_recorderDetails.depthParameters.maxDepth),
        sizeof(int));
    /* intrinsics pixel height and width */
    m_playbackFile.read(
        reinterpret_cast<char *>(&m_recorderDetails.intrinsics.pixelHeight),
        sizeof(float));
    m_playbackFile.read(
        reinterpret_cast<char *>(&m_recorderDetails.intrinsics.pixelWidth),
        sizeof(float));
    /* intrinsics fx, fy, cx and cy */
    float fx;
    m_playbackFile.read(reinterpret_cast<char *>(&fx), sizeof(float));
    float fy;
    m_playbackFile.read(reinterpret_cast<char *>(&fy), sizeof(float));
    float cx;
    m_playbackFile.read(reinterpret_cast<char *>(&cx), sizeof(float));
    float cy;
    m_playbackFile.read(reinterpret_cast<char *>(&cy), sizeof(float));
    /* intrinsics distorsion coefs k1, k2, p1, p2, k3 */
    float k1;
    m_playbackFile.read(reinterpret_cast<char *>(&k1), sizeof(float));
    float k2;
    m_playbackFile.read(reinterpret_cast<char *>(&k2), sizeof(float));
    float p1;
    m_playbackFile.read(reinterpret_cast<char *>(&p1), sizeof(float));
    float p2;
    m_playbackFile.read(reinterpret_cast<char *>(&p2), sizeof(float));
    float k3;
    m_playbackFile.read(reinterpret_cast<char *>(&k3), sizeof(float));

    int sizeOfHeader = (int)m_playbackFile.tellg();
    int sizeOfFrame;
    if (rgbEnabled) {
        sizeOfFrame = sizeof(uint16_t) * (frameHeight * frameWidth +
                                          rgbFrameHeight * rgbFrameWidth);
    } else {
        sizeOfFrame = sizeof(uint16_t) * frameHeight * frameWidth;
    }

    m_numberOfFrames = (fileSize - sizeOfHeader) / sizeOfFrame;

    m_frameDetails.height = frameHeight;
    m_frameDetails.width = frameWidth;
    m_frameDetails.rgbHeight = rgbFrameHeight;
    m_frameDetails.rgbWidth = rgbFrameWidth;
    m_frameDetails.fullDataHeight = frameHeight * 2;
    m_frameDetails.fullDataWidth = frameWidth;
    m_rgbEnabled = rgbEnabled;
    if (m_rgbEnabled) {
        m_frameDetails.type = "depth_ir_rgb";
    } else {
        m_frameDetails.type = "depth_ir";
    }

    m_playbackThreadStop = false;
    m_playBackEofReached = false;
    m_playbackThread =
        std::thread(std::bind(&AditofDemoRecorder::playbackThread, this));

    return m_numberOfFrames;
}

void AditofDemoRecorder::stopPlayback() {
    m_playbackThreadStop = true;
    std::unique_lock<std::mutex> lock(m_playbackMutex);
    m_shouldReadNewFrame = true;
    lock.unlock();
    m_playbackCv.notify_one();
    if (m_playbackThread.joinable()) {
        m_playbackThread.join();
    }
    m_playbackFile.close();
}

void AditofDemoRecorder::recordNewFrame(std::shared_ptr<aditof::Frame> frame) {
    m_recordQueue.enqueue(frame);
}

std::shared_ptr<aditof::Frame> AditofDemoRecorder::readNewFrame() {
    return m_playbackQueue.dequeue();
}

void AditofDemoRecorder::requestFrame() {
    std::unique_lock<std::mutex> lock(m_playbackMutex);
    m_shouldReadNewFrame = true;
    lock.unlock();
    m_playbackCv.notify_one();
}

bool AditofDemoRecorder::isRecordingEnabled() const {
    return !m_recordTreadStop;
}

bool AditofDemoRecorder::isPlaybackEnabled() const {
    return !m_playbackThreadStop;
}

bool AditofDemoRecorder::isPlaybackFinished() const {
    return m_playBackEofReached;
}

int AditofDemoRecorder::getNumberOfFrames() const { return m_numberOfFrames; }

void AditofDemoRecorder::recordThread() {
    while (!m_recordTreadStop) {

        if (!m_recordFile.is_open()) {
            break;
        }

        if (m_recordQueue.empty()) {
            continue;
        }

        auto frame = m_recordQueue.dequeue();

        uint16_t *data;
        frame->getData(aditof::FrameDataType::FULL_DATA, &data);

        unsigned int width = m_frameDetails.width;
        unsigned int height = m_frameDetails.height;
        unsigned int rgbWidth = m_frameDetails.rgbWidth;
        unsigned int rgbHeight = m_frameDetails.rgbHeight;
        int size;

        if (m_rgbEnabled) {
            size = static_cast<int>(
                sizeof(uint16_t) * (width * height * 2 + rgbWidth * rgbHeight));
        } else {
            size = static_cast<int>(sizeof(uint16_t) * width * height * 2);
        }
        m_recordFile.write(reinterpret_cast<const char *>(data), size);
    }
}

void AditofDemoRecorder::playbackThread() {
    while (!m_playbackThreadStop) {

        if (!m_playbackFile.is_open()) {
            break;
        }

        std::unique_lock<std::mutex> lock(m_playbackMutex);
        m_playbackCv.wait(lock, [&]() { return m_shouldReadNewFrame; });
        m_shouldReadNewFrame = false;

        if (m_playbackThreadStop) {
            break;
        }

        std::shared_ptr<aditof::Frame> frame =
            std::make_shared<aditof::Frame>();
        ;
        frame->setDetails(m_frameDetails);
        uint16_t *frameDataLocation;
        frame->getData(aditof::FrameDataType::FULL_DATA, &frameDataLocation);
        unsigned int width = m_frameDetails.width;
        unsigned int height = m_frameDetails.height;
        unsigned int rgbWidth = m_frameDetails.rgbWidth;
        unsigned int rgbHeight = m_frameDetails.rgbHeight;
        uint8_t rgbEnabled = m_rgbEnabled;
        size_t size;
        if (rgbEnabled) {
            size = static_cast<int>(
                sizeof(uint16_t) * (width * height * 2 + rgbWidth * rgbHeight));
        } else {
            size = static_cast<int>(sizeof(uint16_t) * width * height * 2);
        }

        if (m_playbackFile.eof()) {
            memset(frameDataLocation, 0, size);
            m_playBackEofReached = true;
        } else {
            m_playbackFile.read(reinterpret_cast<char *>(frameDataLocation),
                                size);
        }

        m_playbackQueue.enqueue(frame);
    }
}

int AditofDemoRecorder::getRangeMax() const {
    return m_recorderDetails.depthParameters.maxDepth;
}

int AditofDemoRecorder::getRangeMin() const {
    return m_recorderDetails.depthParameters.minDepth;
}

int AditofDemoRecorder::getBitCount() const {
    return m_recorderDetails.bitCount;
}
