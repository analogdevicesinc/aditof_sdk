#include "basic_gui.h"

#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "flyingPixelFilter.h"
#include "guidedFilter.h"

Basic_GUI::Basic_GUI(const std::string &name) : m_viewName(name) {
    m_renderDepth = 0;
    m_varianceFilter = 0;
    m_guidedFilter = 0;
    m_flyingPixelFilter = 0;

    m_nearModeChecked = false;
    m_mediumModeChecked = false;
    m_farModeChecked = true;
    m_modeCurrentValue = 1;
}

Basic_GUI::~Basic_GUI() {}

void Basic_GUI::setFrame(aditof::Frame capturedframe) {
    m_capturedFrame = std::make_shared<aditof::Frame>(capturedframe);
}

void Basic_GUI::initRendering() {

    const cv::String windows[] = {m_viewName, WINDOW_DEPTH, WINDOW_VARIANCE_F,
                                  WINDOW_GUIDED_F, WINDOW_FLYINGPIXEL_F};
    cvui::init(windows, 1);
}

void Basic_GUI::setRange(int range) { m_range = range; }

void Basic_GUI::renderDepthImage() {
    std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;

    uint16_t *depthData;
    localFrame->getData(aditof::FrameDataType::DEPTH, &depthData);

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);

    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    m_depthImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, depthData);
    m_depthImage.convertTo(m_depthImage, CV_8U, 255.0 / m_range);
    flip(m_depthImage, m_depthImage, 1);

    cv::Mat depthColored = m_depthImage;

    cv::applyColorMap(depthColored, depthColored, cv::COLORMAP_RAINBOW);
    cvui::imshow(WINDOW_DEPTH, depthColored);
}

void Basic_GUI::applyVarianceFilter() {
    cv::Mat depthCopy;
    m_depthImage.copyTo(depthCopy);

    cv::Mat outDepth;
    depthCopy.convertTo(outDepth, CV_32FC1);

    cv::Mat muI;
    cv::blur(outDepth, muI, cv::Size(m_filterSize, m_filterSize));

    cv::Mat muI2;
    muI2 = muI.mul(muI);

    cv::Mat IxP;
    cv::blur(outDepth.mul(outDepth), IxP, cv::Size(m_filterSize, m_filterSize));

    cv::Mat sigmaISq;
    sigmaISq = IxP - muI2;

    for (int i = 0; i < outDepth.rows; i++) {
        for (int j = 0; j < outDepth.cols; j++) {
            if (sigmaISq.at<float>(i, j) > m_threshold)
                outDepth.at<float>(i, j) = 0;
        }
    }

    outDepth.convertTo(depthCopy, CV_8U);
    cv::applyColorMap(depthCopy, depthCopy, cv::COLORMAP_RAINBOW);
    cvui::imshow(WINDOW_VARIANCE_F, depthCopy);
}

void Basic_GUI::applyGuidedFilter() {
    cv::Mat depthFiltered = guidedDepthFilter(m_depthImage, m_filterSize,
                                              static_cast<float>(m_eps));

    depthFiltered.convertTo(depthFiltered, CV_8U);
    cv::applyColorMap(depthFiltered, depthFiltered, cv::COLORMAP_RAINBOW);
    cvui::imshow(WINDOW_GUIDED_F, depthFiltered);
}

void Basic_GUI::applyFlyingPixelFilter() {
    cv::Mat depthCopy;
    m_depthImage.copyTo(depthCopy);

    cv::Mat contours;
    cv::Canny(depthCopy, contours, 10, 100);

    flyingPixelFilter fPF(depthCopy.rows, depthCopy.cols, m_filterSizeXi,
                          m_filterSizeYi, m_edgeThi, m_epsEdgei);
    fPF.filter(depthCopy, contours);
    cv::applyColorMap(depthCopy, depthCopy, cv::COLORMAP_RAINBOW);
    cvui::imshow(WINDOW_FLYINGPIXEL_F, depthCopy);
}

int Basic_GUI::getMode() {
    if (m_checkboxChanged) {
        return (2 - static_cast<int>(std::log2(m_modeCurrentValue)));
    }

    return 3;
}

int Basic_GUI::renderCyclic() {
    cv::Mat frame = cv::Mat(500, 500, CV_8UC3);
    frame = cv::Scalar(49, 52, 49);

    cvui::text(frame, 175, 25, "Welcome", 1.2);

    m_checkboxChanged = false;

    // Mode checkbox group
    int btnGroupMode =
        m_nearModeChecked << 2 | m_mediumModeChecked << 1 | m_farModeChecked;
    if (m_modeCurrentValue != btnGroupMode) {
        int xorValue = m_modeCurrentValue ^ btnGroupMode;
        m_modeCurrentValue = xorValue;
        m_nearModeChecked = xorValue & (1 << 2);
        m_mediumModeChecked = xorValue & (1 << 1);
        m_farModeChecked = xorValue & 1;
        m_checkboxChanged = true;
    }

    cvui::beginColumn(frame, 50, 400);
    cvui::space(10);
    cvui::text("Mode: ", 0.6);
    cvui::space(10);
    cvui::beginRow(frame, 50, 450);
    cvui::checkbox("Near", &m_nearModeChecked);
    cvui::space(10);
    cvui::checkbox("Medium", &m_mediumModeChecked);
    cvui::space(10);
    cvui::checkbox("Far", &m_farModeChecked);
    cvui::endRow();
    cvui::endColumn();

    if (cvui::button(frame, 50, 80, 100, 40,
                     !m_renderDepth ? "Render Depth" : "Stop Depth")) {
        if (m_renderDepth == 0)
            m_renderDepth = 1;
        else
            m_renderDepth = 0;
    }

    if (m_renderDepth) {
        if (cvui::button(frame, 50, 150, 100, 30,
                         !m_varianceFilter ? "Variance filter"
                                           : "Stop filtering")) {

            if (m_varianceFilter == 0)
                m_varianceFilter = 1;
            else
                m_varianceFilter = 0;
        }
        if (m_varianceFilter) {
            cvui::text(frame, 200, 150, "Threshold:", 0.5);
            cvui::counter(frame, 200, 170, &m_threshold);
        }

        if (cvui::button(frame, 50, 210, 100, 30,
                         !m_guidedFilter ? "Guided filter"
                                         : "Stop filtering")) {
            if (m_guidedFilter == 0)
                m_guidedFilter = 1;
            else
                m_guidedFilter = 0;
        }
        if (m_guidedFilter) {
            cvui::text(frame, 200, 210, "Epsilon:", 0.5);
            cvui::counter(frame, 200, 230, &m_eps, 2.0);
        }

        if (m_varianceFilter || m_guidedFilter) {
            cvui::text(frame, 200, 80, "FilterSize:", 0.5);
            cvui::counter(frame, 200, 100, &m_filterSize);
        }

        if (cvui::button(frame, 50, 270, 100, 30,
                         !m_flyingPixelFilter ? "FlyingPixel filter"
                                              : "Stop filtering")) {
            if (m_flyingPixelFilter == 0)
                m_flyingPixelFilter = 1;
            else
                m_flyingPixelFilter = 0;
        }
        if (m_flyingPixelFilter) {
            cvui::text(frame, 200, 270, "EdgeThi:", 0.5);
            cvui::counter(frame, 200, 290, &m_edgeThi, 2.0);

            cvui::text(frame, 350, 270, "EpsEdgei:", 0.5);
            cvui::counter(frame, 350, 290, &m_epsEdgei, 2.0);

            cvui::text(frame, 200, 320, "FilterSizeX:", 0.5);
            cvui::counter(frame, 200, 340, &m_filterSizeXi);

            cvui::text(frame, 350, 320, "FilterSizeY:", 0.5);
            cvui::counter(frame, 350, 340, &m_filterSizeYi);
        }

    } else {
        m_varianceFilter = 0;
        m_guidedFilter = 0;
        m_flyingPixelFilter = 0;
    }

    if (cvui::button(frame, 350, 80, 100, 40, "Exit")) {
        std::cout << "Exit button clicked!" << std::endl;
        return 0;
    }

    cvui::update();
    cv::imshow(m_viewName, frame);

    if (m_renderDepth) {
        renderDepthImage();
    } else {
        cv::destroyWindow(WINDOW_DEPTH);
    }

    if (m_varianceFilter) {
        applyVarianceFilter();
    } else {
        cv::destroyWindow(WINDOW_VARIANCE_F);
    }

    if (m_guidedFilter) {
        applyGuidedFilter();
    } else {
        cv::destroyWindow(WINDOW_GUIDED_F);
    }

    if (m_flyingPixelFilter) {
        applyFlyingPixelFilter();
    } else {
        cv::destroyWindow(WINDOW_FLYINGPIXEL_F);
    }

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
        return 0;
    }

    return 1;
}
