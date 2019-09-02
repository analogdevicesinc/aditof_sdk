#include "basic_gui.h"

#include <sstream>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

Basic_GUI::Basic_GUI(const std::string &name) : m_viewName(name) {
    renderDepth = 0;
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

    uint16_t *data;
    localFrame->getData(aditof::FrameDataType::DEPTH, &data);

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);

    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    m_depthImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, data);
    m_depthImage.convertTo(m_depthImage, CV_8U, 255.0 / 5999);
    applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_RAINBOW);
    flip(m_depthImage, m_depthImage, 1);

    cvui::imshow(WINDOW1_NAME, m_depthImage);
}

int Basic_GUI::renderCyclic() {

    cv::Mat frame = cv::Mat(400, 400, CV_8UC3);
    frame = cv::Scalar(49, 52, 49);

    cvui::text(frame, 125, 25, "Welcome", 1.2);

    if (cvui::button(frame, 50, 100, 100, 40, "Render Depth")) {
        renderDepth = 1;
    }

    // Render a regular button.
    if (cvui::button(frame, 250, 350, 100, 40, "Exit")) {
        std::cout << "Exit button clicked!" << std::endl;
        return 0;
    }

    cvui::update();
    cv::imshow(m_viewName, frame);
    if (renderDepth) {
        renderDepthImage();
    }

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
        return 0;
    }

    return 1;
}
