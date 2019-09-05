#include "basic_gui.h"

#include <sstream>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

Basic_GUI::Basic_GUI(const std::string &name) : m_viewName(name) {
    m_renderDepth = 0;
    m_renderIR = 0;
    m_detectEdges = 0;
    m_detectObjects = 0;
}
Basic_GUI::~Basic_GUI() {}

void Basic_GUI::setFrame(aditof::Frame capturedframe) {
    m_capturedFrame = std::make_shared<aditof::Frame>(capturedframe);
}

void Basic_GUI::renderOnce() {

    const cv::String windows[] = {m_viewName, WINDOW1_NAME, WINDOW2_NAME,
                                  WINDOW3_NAME, WINDOW4_NAME};
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

void Basic_GUI::detectEdgesfromDepth() {
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

void Basic_GUI::detectObjectsfromDepth() {
    std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;

    uint16_t *depthData;
    localFrame->getData(aditof::FrameDataType::DEPTH, &depthData);

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);

    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    m_depthImage = cv::Mat(frameHeight, frameWidth, CV_16UC1, depthData);
    m_depthImage.convertTo(m_depthImage, CV_8U, 255.0 / 5999);
    flip(m_depthImage, m_depthImage, 1);

    cv::Mat udisparity = computeUdisparity(m_depthImage, 255);
    cv::Mat threshold = computeThreshold(udisparity, 10);
    computeConnectedComponentsUDisp(m_depthImage, threshold, 3);

    cvui::imshow(WINDOW4_NAME, threshold);
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
                     !m_renderDepth ? "Render Depth" : "Stop Depth")) {
        if (m_renderDepth == 0)
            m_renderDepth = 1;
        else
            m_renderDepth = 0;
    }

    if (m_renderDepth) {
        if (cvui::button(frame, 50, 200, 100, 40,
                         !m_detectEdges ? "Detect edges" : "Stop Detection")) {
            if (m_detectEdges == 0)
                m_detectEdges = 1;
            else
                m_detectEdges = 0;
        }
        if (cvui::button(frame, 250, 200, 100, 40,
                         !m_detectObjects ? "Detect Objects"
                                          : "Stop Detection")) {
            if (m_detectObjects == 0)
                m_detectObjects = 1;
            else
                m_detectObjects = 0;
        }
    } else {
        m_detectEdges = 0;
        m_detectObjects = 0;
    }

    if (cvui::button(frame, 250, 100, 100, 40,
                     !m_renderIR ? "Render IR" : "Stop IR")) {
        if (m_renderIR == 0)
            m_renderIR = 1;
        else
            m_renderIR = 0;
    }

    if (cvui::button(frame, 250, 350, 100, 40, "Exit")) {
        std::cout << "Exit button clicked!" << std::endl;
        return 0;
    }

    cvui::update();
    cv::imshow(m_viewName, frame);
    if (m_renderDepth) {
        renderDepthImage();
    } else {
        cv::destroyWindow(WINDOW1_NAME);
    }

    if (m_renderIR) {
        renderIRImage();
    } else {
        cv::destroyWindow(WINDOW2_NAME);
    }

    if (m_detectEdges) {
        detectEdgesfromDepth();
    } else {
        cv::destroyWindow(WINDOW3_NAME);
    }

    if (m_detectObjects) {
        detectObjectsfromDepth();
    } else {
        cv::destroyWindow(WINDOW4_NAME);
        cv::destroyWindow(WINDOW5_NAME);
    }

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
        return 0;
    }

    return 1;
}

cv::Mat Basic_GUI::computeThreshold(cv::Mat ImgToProcess, int const_value) {
    cv::Size size =
        cv::Size(ImgToProcess.size().width, ImgToProcess.size().height);
    cv::Mat result = cv::Mat::zeros(size, CV_8U);
    ;
    for (int i = 0; i < ImgToProcess.size().height; i++) {
        for (int j = 0; j < ImgToProcess.size().width; j++) {
            if ((static_cast<int>(ImgToProcess.at<unsigned char>(i, j))) >
                const_value)
                result.at<unsigned char>(i, j) = 255;
            else
                result.at<unsigned char>(i, j) = 0;
            // if (j > 200) // take awat background and far away objects
            //   result.at<unsigned char>(i, j) = 0;
        }
    }
    return result;
}

cv::Mat Basic_GUI::computeUdisparity(cv::Mat ImgToProcess, int max_disp) {
    cv::Size size = cv::Size(ImgToProcess.size().width, max_disp);
    cv::Mat result = cv::Mat::zeros(size, CV_8U);
    for (int j = 0; j < ImgToProcess.size().width; j++) {
        for (int i = 0; i < ImgToProcess.size().height; i++) {
            int disp = static_cast<int>(ImgToProcess.at<unsigned char>(i, j));
            if (disp > 0 && disp < 100)
                result.at<unsigned char>(disp, j)++;
        }
    }
    return result;
}

cv::Mat Basic_GUI::computeVdisparity(cv::Mat ImgToProcess, int max_disp) {
    cv::Size size = cv::Size(max_disp, ImgToProcess.size().height);
    cv::Mat result = cv::Mat::zeros(size, CV_8U);
    for (int j = 0; j < ImgToProcess.size().width; j++) {
        for (int i = 0; i < ImgToProcess.size().height; i++) {
            int disp = static_cast<int>(ImgToProcess.at<unsigned char>(i, j));
            if (disp > 0 && disp < 100)
                result.at<unsigned char>(i, disp)++;
        }
    }
    return result;
}

void Basic_GUI::labelCompute(cv::Mat img, std::vector<std::vector<int>> &labels,
                             int i, int j, int dist, int labelNo,
                             std::vector<cv::Point2i> &blob) {
    if (img.at<unsigned char>(i, j) == 0)
        return;
    if (labels[i][j] != 0) {
        return;
    }
    labels[i][j] = labelNo;
    blob.push_back(cv::Point2i(j, i));
    for (int k = i - dist; k < i + dist; k++) {
        for (int l = j - dist; l < j + dist; l++) {
            if (k >= 0 && k < img.size().height && l >= 0 &&

                l < img.size().width) {
                labelCompute(img, labels, k, l, dist, labelNo, blob);
            }
        }
    }
}

void Basic_GUI::labelImage(cv::Mat img, std::vector<std::vector<int>> &labels,
                           int dist,
                           std::vector<std::vector<cv::Point2i>> &blobs) {
    int labelNo = 0;
    std::vector<cv::Point2i> blob;
    for (int i = 0; i < img.size().height; i++) {
        for (int j = 0; j < img.size().width; j++) {
            if (img.at<unsigned char>(i, j) == 0) {
                continue;
            }
            if (labels[i][j] != 0) {
                continue;
            }
            labelNo++;
            labelCompute(img, labels, i, j, dist, labelNo, blob);
            blobs.push_back(blob);
            blob.clear();
        }
    }
}

void Basic_GUI::computeConnectedComponentsUDisp(cv::Mat input,
                                                cv::Mat threshold, int dist) {
    std::vector<std::vector<cv::Point2i>> blobs;
    cv::Mat outputColor = cv::Mat::zeros(input.size(), CV_8UC3);
    cv::Mat label_image;
    cv::Mat output = cv::Mat::zeros(threshold.size(), CV_8UC3);

    threshold.convertTo(label_image, CV_8U);

    int height = threshold.size().height;
    int width = threshold.size().width;
    std::vector<std::vector<int>> labels;

    labels.resize(static_cast<unsigned int>(height));
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            labels[i].resize(static_cast<unsigned int>(width));
            labels[i][j] = 0;
        }
    }
    labelImage(label_image, labels, dist, blobs);
    int valdisp;
    int colorIndex = 0;
    cv::Vec3b color;
    for (size_t i = 0; i < blobs.size(); i++) {
        if (NUMBEROFCOLORS > 0 && !m_colorsVector.empty()) {
            color = m_colorsVector.at(colorIndex);

            if (colorIndex >= NUMBEROFCOLORS - 1) {
                colorIndex = 0;
            } else {
                colorIndex++;
            }
        } else {
            color = cv::Vec3b(0, 0, 0);
        }

        if (blobs[i].size() < MINSIZEOFBLOB_NEAR) {
            continue;
        } else {
            for (size_t j = 0; j < blobs[i].size(); j++) {
                int y = blobs[i][j].x;
                int x = blobs[i][j].y;

                output.at<cv::Vec3b>(x, y) = color;

                for (int line = 0; line < input.size().height; line++) {
                    valdisp =
                        static_cast<int>(input.at<unsigned char>(line, y));
                    if (valdisp == x) {
                        outputColor.at<cv::Vec3b>(line, y) = color;
                    }
                }
            }
        }
    }
    cvui::imshow(WINDOW5_NAME, outputColor);
}

void Basic_GUI::generateColorsVector() {
    m_colorsVector.resize(static_cast<unsigned int>(NUMBEROFCOLORS));
    for (int i = 0; i < NUMBEROFCOLORS; i++) {
        unsigned char r =
            static_cast<unsigned char>(255 * (rand() / (1.0 + RAND_MAX)));
        unsigned char g =
            static_cast<unsigned char>(255 * (rand() / (1.0 + RAND_MAX)));
        unsigned char b =
            static_cast<unsigned char>(255 * (rand() / (1.0 + RAND_MAX)));
        cv::Vec3b colorTuple(b, g, r);
        m_colorsVector.at(i) = colorTuple;
    }
}
