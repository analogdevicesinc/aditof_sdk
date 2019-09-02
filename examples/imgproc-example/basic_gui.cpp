#include "basic_gui.h"

#include <sstream>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

Basic_GUI::Basic_GUI(const std::string &name) : m_viewName(name) {
    renderDepth = 0;
    renderIR = 0;
    computeStuff = 0;
}
Basic_GUI::~Basic_GUI() {}

void Basic_GUI::setFrame(aditof::Frame capturedframe) {
    m_capturedFrame = std::make_shared<aditof::Frame>(capturedframe);
}

void Basic_GUI::renderOnce() {

    const cv::String windows[] = {m_viewName, WINDOW1_NAME, WINDOW2_NAME,
                                  WINDOW3_NAME};
    cvui::init(windows, 1);
}

void Basic_GUI::renderDepthImage() {
    std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;

    uint16_t *depthData;
    localFrame->getData(aditof::FrameDataType::DEPTH, &depthData);

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);

    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    m_depthImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, depthData);
    m_depthImage.convertTo(m_depthImage, CV_8U, 255.0 / 5999);
    applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_RAINBOW);
    flip(m_depthImage, m_depthImage, 1);

    cvui::imshow(WINDOW1_NAME, m_depthImage);
}

void Basic_GUI::computeStuffOnImage() {
    std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;

    uint16_t *depthData;
    localFrame->getData(aditof::FrameDataType::DEPTH, &depthData);

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);

    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    m_depthImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, depthData);
    m_depthImage.convertTo(m_depthImage, CV_8U, 255.0 / 5999);
    applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_RAINBOW);
    flip(m_depthImage, m_depthImage, 1);

    cv::Mat gray_image;
    cv::Mat contours;
    cv::cvtColor(m_depthImage, gray_image, cv::COLOR_RGB2GRAY);
    cv::Canny(gray_image, contours, 10, 350);

    cvui::imshow(WINDOW3_NAME, contours);
}

void Basic_GUI::renderIRImage() {
    std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;

    uint16_t *irData;
    localFrame->getData(aditof::FrameDataType::IR, &irData);

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);

    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    m_irImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, irData);
    m_irImage.convertTo(m_irImage, CV_8U, 255.0 / 5999);
    flip(m_irImage, m_irImage, 1);
    cvui::imshow(WINDOW2_NAME, m_irImage);
}

int Basic_GUI::renderCyclic() {

    cv::Mat frame = cv::Mat(400, 400, CV_8UC3);
    frame = cv::Scalar(49, 52, 49);

    cvui::text(frame, 125, 25, "Welcome", 1.2);

    if (cvui::button(frame, 50, 100, 100, 40,
                     !renderDepth ? "Render Depth" : "Stop Depth")) {
        if (renderDepth == 0)
            renderDepth = 1;
        else
            renderDepth = 0;
    }

    if (renderDepth) {
        if (cvui::button(frame, 50, 200, 100, 40,
                         !computeStuff ? "Compute stuff" : "Stop Compute")) {
            if (computeStuff == 0)
                computeStuff = 1;
            else
                computeStuff = 0;
        }
    } else
        computeStuff = 0;

    if (cvui::button(frame, 250, 100, 100, 40,
                     !renderIR ? "Render IR" : "Stop IR")) {
        if (renderIR == 0)
            renderIR = 1;
        else
            renderIR = 0;
    }

    if (cvui::button(frame, 250, 350, 100, 40, "Exit")) {
        std::cout << "Exit button clicked!" << std::endl;
        return 0;
    }

    cvui::update();
    cv::imshow(m_viewName, frame);
    if (renderDepth) {
        renderDepthImage();
    } else {
        cv::destroyWindow(WINDOW1_NAME);
    }

    if (renderIR) {
        renderIRImage();
    } else {
        cv::destroyWindow(WINDOW2_NAME);
        m_depthImage.release();
    }

    if (computeStuff) {
        computeStuffOnImage();
    } else {
        cv::destroyWindow(WINDOW3_NAME);
    }
    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
        return 0;
    }

    return 1;
}
