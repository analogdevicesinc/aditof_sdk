#include "basic_gui.h"

#include <sstream>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

Basic_GUI::Basic_GUI(const std::string &name) : m_viewName(name) {
    m_renderDepth = 0;
    m_picture_work = 0;
    m_renderIR = 0;
    m_detectEdges = 0;
    m_distanceVal = 3000;
    m_Theta1 = 10;
    m_Theta2 = 100;
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

void Basic_GUI::setDistanceValue(int distanceVal) {
    m_distanceVal = distanceVal;
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
    m_depthImage.convertTo(m_depthImage, CV_8U, 255.0 / m_distanceVal);
    applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_RAINBOW);
    flip(m_depthImage, m_depthImage, 1);

    cvui::imshow(WINDOW1_NAME, m_depthImage);
}

void Basic_GUI::detectEdgesfromDepth() {
    std::shared_ptr<aditof::Frame> localFrame = m_capturedFrame;

    aditof::FrameDetails frameDetails;
    localFrame->getDetails(frameDetails);
    int frameHeight = static_cast<int>(frameDetails.height) / 2;
    int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *depthData;
    localFrame->getData(aditof::FrameDataType::DEPTH, &depthData);
    cv::Mat depth_map;
    depth_map = cv::Mat(frameHeight, frameWidth, CV_16UC1, depthData);
    depth_map.convertTo(depth_map, CV_8U, 255.0 / m_distanceVal);
    flip(depth_map, depth_map, 1);
    applyColorMap(depth_map, depth_map, cv::COLORMAP_RAINBOW);
    cv::imwrite("pic/depth.png", depth_map);

    uint16_t *irData;
    localFrame->getData(aditof::FrameDataType::IR, &irData);
    cv::Mat ir_map;
    ir_map = cv::Mat(frameHeight, frameWidth, CV_16UC1, irData);
    ir_map.convertTo(ir_map, CV_8U, 255.0 / m_distanceVal);
    flip(ir_map, ir_map, 1);
    cv::cvtColor(ir_map, ir_map, cv::COLOR_GRAY2RGB);
    cv::imwrite("pic/ir.png", ir_map);

    cv::Mat result;
    cv::addWeighted(ir_map, 0.4, depth_map, 0.6, 0, result);
    cvui::imshow("combined", result);

    cv::imwrite("pic/result.png", result);

    cv::Mat gray_image;
    cv::cvtColor(ir_map, gray_image, cv::COLOR_RGB2GRAY);
    cvui::imshow("gray ", gray_image);

    cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
    cvui::imshow("gray blured", gray_image);

    // nu merge cu thershold caci sunt prea apropiate nunatele de gri!
    //    cv::Mat thresh;
    //    int dilation_size = 2;
    //    cv::threshold(gray_image, thresh, 150, 255, cv::THRESH_BINARY);
    //    cv::Mat element = cv::getStructuringElement(
    //        cv::MORPH_CROSS, cv::Size(2 * dilation_size + 1, 2 * dilation_size
    //        + 1), cv::Point(dilation_size, dilation_size));
    //    cvui::imshow("thresh", thresh);
    //    cv::erode(thresh, thresh, element);
    //    cvui::imshow("erode", thresh);
    //    cv::dilate(thresh, thresh, element);
    //    cvui::imshow("dilate", thresh);

    cv::Mat canny;
    cv::Canny(gray_image, canny, m_Theta1, m_Theta2);
    cvui::imshow("canny", canny);
    cv::imwrite("pic/canny.png", canny);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny, contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    cv::RNG rng(12345);
    cv::Mat drawing;
    result.copyTo(drawing);
    // cv::Scalar color = cv::Scalar(0, 255, 0);
    for (int i = 0; i < contours.size(); i++) {

        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                      rng.uniform(0, 255));
        if (contours.at(i).size() < 100)
            continue;
        cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0,
                         cv::Point());
    }
    cvui::imshow("contours", drawing);
    cv::imwrite("pic/contours.png", drawing);
}

void Basic_GUI::detectEdgesfromDepthImage() {

    std::fstream fout;

    // opens an existing csv file or creates a new file.
    fout.open("points.csv", std::ios::out);

    cv::Mat depth_map = cv::imread("pic/depth.png");

    cv::Mat ir_map = cv::imread("pic/ir.png");

    cv::Mat result = cv::imread("pic/result.png");

    cv::Mat gray_image;
    cv::cvtColor(ir_map, gray_image, cv::COLOR_RGB2GRAY);
    cvui::imshow("gray ", gray_image);

    cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
    cvui::imshow("gray blured", gray_image);

    cv::Mat canny;
    cv::Canny(gray_image, canny, m_Theta1, m_Theta2);
    cvui::imshow("canny", canny);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny, contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    cv::RNG rng(12345);
    cv::Mat drawing;
    result.copyTo(drawing);
    std::cout << "Sizes for countours:" << std::endl;
    cv::Scalar color = cv::Scalar(0, 255, 0);
    std::sort(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
            return a.size() > b.size();
        });

    std::vector<cv::Point> merged(contours.at(0));
    merged.insert(merged.end(), contours.at(1).begin(), contours.at(1).end());

    std::sort(merged.begin(), merged.end(),
              [](const cv::Point &a, const cv::Point &b) { return a.y < b.y; });

    std::vector<cv::Point> x_sorted(merged);
    std::sort(x_sorted.begin(), x_sorted.end(),
              [](const cv::Point &a, const cv::Point &b) { return a.x < b.x; });

    int min_x = x_sorted.at(0).x;
    int max_x = x_sorted.at(x_sorted.size() - 1).x;
    int middle_x = static_cast<int>((min_x + max_x) * 0.5f);
    int err_lvl = 20;

    // cv::polylines(drawing, merged, true, cv::Scalar(255, 255, 255));

    cv::Point leftp = cv::Point(0, 0);
    cv::Point rightp = cv::Point(0, 0);
    cv::Point centerp = cv::Point(0, 0);
    cv::Point bottomp = cv::Point(0, 0);

    std::cout << leftp << " " << bottomp << std::endl;
    std::cout << "urm punctele:\n";

    for (auto i : merged) {

        if (i.x < min_x + err_lvl) {
            leftp = i;
            std::cout << leftp << '\n';
            break;
        }
    }
    for (auto i : merged) {

        if (i.x > max_x - err_lvl) {
            rightp = i;
            std::cout << rightp << '\n';
            break;
        }
    }

    std::reverse(merged.begin(), merged.end());
    for (auto i : merged) {

        if (i.x > middle_x - err_lvl && i.x < middle_x + err_lvl) {
            bottomp = i;
            std::cout << bottomp << '\n';
            break;
        }
    }

    std::reverse(merged.begin(), merged.end());
    for (auto i : merged) {

        if (i.x > bottomp.x - 1 && i.x < bottomp.x + 1) {
            centerp = i;
            std::cout << centerp << '\n';
            break;
        }
    }
    cv::line(drawing, leftp, centerp, cv::Scalar(255, 255, 255));
    cv::line(drawing, centerp, rightp, cv::Scalar(255, 255, 255));
    cv::line(drawing, centerp, bottomp, cv::Scalar(255, 255, 255));

    double AB = sqrt((leftp.x - centerp.x) * (leftp.x - centerp.x) +
                     (leftp.y - centerp.y) * (leftp.y - centerp.y));
    double AC = sqrt((leftp.x - rightp.x) * (leftp.x - rightp.x) +
                     (leftp.y - rightp.y) * (leftp.y - rightp.y));
    double BC = sqrt((centerp.x - rightp.x) * (centerp.x - rightp.x) +
                     (centerp.y - rightp.y) * (centerp.y - rightp.y));
    std::cout << "AB: " << AB << '\n';
    std::cout << "AC: " << AC << '\n';
    std::cout << "BC: " << BC << '\n';
    double cos_theta = (AB * AB - AC * AC + BC * BC) / (2 * AB * BC);
    double PI = 3.14159265;
    double theta = acos(cos_theta) * 180.0 / PI;

    std::cout << "theta: " << theta << '\n';

    //    for (int i = 0; i < contours.size(); i++) {
    //        std::sort(
    //        contours.at(i).begin(), contours.at(i).end(),
    //        [](const cv::Point &a, const cv::Point &b) { return
    //        a.x >
    //        b.x; });
    //        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,
    //        255),
    //                                      rng.uniform(0, 255));
    //        if (contours.at(i).size() < 50)
    //            continue;
    //        cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0,
    //                         cv::Point());
    //    }
    cvui::imshow("contours", drawing);
    cv::imwrite("pic/contours_merged.png", drawing);
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
    m_irImage.convertTo(m_irImage, CV_8U, 255.0 / m_distanceVal);
    flip(m_irImage, m_irImage, 1);
    cvui::imshow(WINDOW2_NAME, m_irImage);
}

int Basic_GUI::renderCyclic() {

    cv::Mat frame = cv::Mat(400, 400, CV_8UC3);
    frame = cv::Scalar(49, 52, 49);

    cvui::text(frame, 125, 25, "Welcome", 1.2);

    if (cvui::button(frame, 30, 100, 100, 40,
                     !m_renderDepth ? "Render Depth" : "Stop Depth")) {
        if (m_renderDepth == 0)
            m_renderDepth = 1;
        else
            m_renderDepth = 0;
    }

    if (cvui::button(frame, 270, 100, 100, 40,
                     !m_picture_work ? "Pic Work" : "Stop Pic Work")) {

        m_picture_work = 1;
    }

    if (m_renderDepth) {
        if (cvui::button(frame, 50, 200, 100, 40,
                         !m_detectEdges ? "Detect contours"
                                        : "Stop Detection")) {
            if (m_detectEdges == 0)
                m_detectEdges = 1;
            else
                m_detectEdges = 0;
        }
        if (m_detectEdges) {
            cvui::text(frame, 50, 250, "Theta1 Canny:", 0.5);
            cvui::counter(frame, 50, 270, &m_Theta1, 10);

            cvui::text(frame, 250, 250, "Theta2 Canny:", 0.5);
            cvui::counter(frame, 250, 270, &m_Theta2, 10);
        }

    } else {
        m_detectEdges = 0;
    }

    if (cvui::button(frame, 150, 100, 100, 40,
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

    if (m_picture_work) {
        detectEdgesfromDepthImage();
        m_picture_work = 0;
    } else {
        cv::destroyWindow(WINDOW3_NAME);
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
