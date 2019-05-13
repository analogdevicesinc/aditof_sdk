#include "aditofdemorecorder.h"

#include <functional>
#include <string.h>

AditofDemoRecorder::AditofDemoRecorder()
    : m_frameDetails{0, 0, ""}, m_recordTreadStop(true),
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

void AditofDemoRecorder::startRecording(const std::string &fileName,
					unsigned int height, unsigned int width,
					unsigned int fps) {
    m_recordFile = std::ofstream(fileName, std::ios::binary);
    m_recordFile.write(reinterpret_cast<const char *>(&height),
		       sizeof(unsigned int));
    m_recordFile.write(reinterpret_cast<const char *>(&width),
		       sizeof(unsigned int));
    m_recordFile.write(reinterpret_cast<const char *>(&fps),
		       sizeof(unsigned int));

    m_frameDetails.height = height;
    m_frameDetails.width = width;

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
    unsigned int height = 0;
    unsigned int width = 0;

    m_playbackFile = std::ifstream(fileName, std::ios::binary);

    m_playbackFile.seekg(0, std::ios_base::end);
    int fileSize = m_playbackFile.tellg();
    m_playbackFile.seekg(0, std::ios_base::beg);

    m_playbackFile.read(reinterpret_cast<char *>(&height), sizeof(int));
    m_playbackFile.read(reinterpret_cast<char *>(&width), sizeof(int));
    m_playbackFile.read(reinterpret_cast<char *>(&fps), sizeof(int));

    int sizeOfHeader = 3 * sizeof(int);
    int sizeOfFrame = sizeof(uint16_t) * height * width;

    m_numberOfFrames = (fileSize - sizeOfHeader) / sizeOfFrame;

    m_frameDetails.height = height;
    m_frameDetails.width = width;

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

int AditofDemoRecorder::getNumberOfFrames() const
{
	return m_numberOfFrames;
}

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
	frame->getData(aditof::FrameDataType::RAW, &data);

	unsigned int width = m_frameDetails.width;
	unsigned int height = m_frameDetails.height;

	int size = static_cast<int>(sizeof(uint16_t) * width * height);

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
	frame->getData(aditof::FrameDataType::RAW, &frameDataLocation);

	unsigned int width = m_frameDetails.width;
	unsigned int height = m_frameDetails.height;

	if (m_playbackFile.eof()) {
	    memset(frameDataLocation, 0, sizeof(uint16_t) * width * height);
	    m_playBackEofReached = true;
	} else {
	    int size = static_cast<int>(sizeof(uint16_t) * width * height);
	    m_playbackFile.read(reinterpret_cast<char *>(frameDataLocation),
				size);
	}

	m_playbackQueue.enqueue(frame);
    }
}
