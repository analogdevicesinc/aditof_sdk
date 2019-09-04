#ifndef BASICGUI_H
#define BASICGUI_H

#include <aditof/frame.h>
#include <fstream>
#include <string>

#include "basic_controller.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif

class Basic_GUI {
  public:
    Basic_GUI(const std::string &name, int numberOfColors);
    ~Basic_GUI();

    void setFrame(aditof::Frame capturedframe);
    void renderOnce();
    int renderCyclic();

    void renderDepthImage();
    void renderIRImage();
    void detectEdgesfromDepth();
    void detectObjectsfromDepth();

    void generateColorsVector();

  private:
    cv::Mat computeUdisparity(cv::Mat ImgToProcess, int max_disp);
    cv::Mat computeVdisparity(cv::Mat ImgToProcess, int max_disp);
    cv::Mat computeThreshold(cv::Mat ImgToProcess, int const_value);
    void labelCompute(cv::Mat img, std::vector<std::vector<int>> &labels, int i,
                      int j, int dist, int labelNo,
                      std::vector<cv::Point2i> &blob);
    void labelImage(cv::Mat img, std::vector<std::vector<int>> &labels,
                    int dist, std::vector<std::vector<cv::Point2i>> &blobs);
    void computeConnectedComponentsUDisp(cv::Mat input, cv::Mat threshold,
                                         int dist);

  private:
    std::shared_ptr<aditof::Frame> m_capturedFrame;
    std::string m_viewName;

    cv::Mat m_depthImage;
    cv::Mat m_irImage;

    const std::string WINDOW1_NAME = "Depth Image";
    const std::string WINDOW2_NAME = "IR Image";
    const std::string WINDOW3_NAME = "Edges";
    const std::string WINDOW4_NAME = "Histogram";
    const std::string WINDOW5_NAME = "Detected Objects";

    int m_distanceVal;

    int m_renderDepth;
    int m_renderIR;
    int m_detectEdges;
    int m_detectObjects;

    std::vector<cv::Vec3b> m_colorsVector;
    int m_numberOfColors;
};

#endif
