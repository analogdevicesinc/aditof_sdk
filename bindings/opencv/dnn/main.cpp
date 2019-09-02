#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
// #include <opencv2/ximgproc/edge_filter.hpp>

#include <cstdlib>
#include <iomanip>
#include <iostream>

#include "../aditof_opencv.h"

const size_t inWidth = 300;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;
const char *classNames[] = {
    "background", "aeroplane",   "bicycle", "bird",  "boat",
    "bottle",     "bus",         "car",     "cat",   "chair",
    "cow",        "diningtable", "dog",     "horse", "motorbike",
    "person",     "pottedplant", "sheep",   "sofa",  "train",
    "tvmonitor"};

int main(int argc, char *argv[]) {

    using namespace aditof;

    cv::dnn::Net net = cv::dnn::readNetFromCaffe(PROTOTXT, MODEL);

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    Status status = Status::OK;

    System system;
    status = system.initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize system!";
        return 0;
    }

    std::vector<Camera *> cameras;
    system.getCameraList(cameras);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found!";
        return 0;
    }

    auto camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return 0;
    }

    status = camera->setFrameType(frameTypes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return 0;
    }

    status = camera->setMode(modes[1]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    aditof::Frame frame;
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 0;
    }

    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);

    cv::Size cropSize;
    if ((float)frameDetails.width / (float)(frameDetails.height / 2) >
        WHRatio) {
        cropSize = cv::Size(
            static_cast<int>((float)(frameDetails.height / 2) * WHRatio),
            (frameDetails.height / 2));
    } else {
        cropSize =
            cv::Size(frameDetails.width,
                     static_cast<int>((float)frameDetails.width / WHRatio));
    }

    cv::Rect crop(cv::Point((frameDetails.width - cropSize.width) / 2,
                            ((frameDetails.height / 2) - cropSize.height) / 2),
                  cropSize);

    // Look up table to adjust image => use gamma correction
    float gamma = 0.4f;
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar *p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);

    while (cv::waitKey(1) != 27 &&
           getWindowProperty("Display Image", cv::WND_PROP_AUTOSIZE) >= 0) {
        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }

        /* Convert from frame to depth mat */
        cv::Mat frameMat;
        status = fromFrameToDepthMat(frame, frameMat);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return 0;
        }

        /* Obtain the depth mat from the frame in order to determine
         * the distance of the identified objects */
        cv::Mat depthMat;
        status = fromFrameToDepthMat(frame, depthMat);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return 0;
        }

        /* Distance factor */
        double distance_scale = 255.0 / 3000;

        /* Convert from raw values to values that opencv can understand */
        frameMat.convertTo(frameMat, CV_8U, distance_scale);

        /* Use the ir mat as input for the Net as we currently don't have
         * an rgb source */
        applyColorMap(frameMat, frameMat, cv::COLORMAP_RAINBOW);
        // LUT(frameMat, lookUpTable, frameMat);
        // frameMat.convertTo(frameMat, -1, 0.8, 0);
        // cv::ximgproc::GuidedFilter guidedFIlter;
        // cv::ximgproc::guidedFilter(frameMat, frameMat);
        // cv::fastNlMeansDenoisingColored(frameMat, frameMat);

        cv::Mat inputBlob =
            cv::dnn::blobFromImage(frameMat, inScaleFactor,
                                   cv::Size(inWidth, inHeight), meanVal, false);

        net.setInput(inputBlob, "data");

        cv::Mat detection = net.forward("detection_out");

        cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F,
                             detection.ptr<float>());

        frameMat = frameMat(crop);

        float confidenceThreshold = 0.4f;

        for (int i = 0; i < detectionMat.rows; i++) {
            float confidence = detectionMat.at<float>(i, 2);

            if (confidence > confidenceThreshold) {
                size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));

                int xLeftBottom = static_cast<int>(
                    detectionMat.at<float>(i, 3) * frameMat.cols);
                int yLeftBottom = static_cast<int>(
                    detectionMat.at<float>(i, 4) * frameMat.rows);
                int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) *
                                                 frameMat.cols);
                int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) *
                                                 frameMat.rows);

                cv::Rect object((int)xLeftBottom, (int)yLeftBottom,
                                (int)(xRightTop - xLeftBottom),
                                (int)(yRightTop - yLeftBottom));

                object = object & cv::Rect(0, 0, frameMat.cols, frameMat.rows);

                auto center = (object.br() + object.tl()) * 0.5;

                std::ostringstream ss;
                ss.str("");
                ss << std::setprecision(3)
                   << depthMat.at<ushort>(center) / 1000.0 * 0.3 << " meters";
                cv::String conf(ss.str());

                cv::rectangle(frameMat, object, cv::Scalar(0, 255, 0));
                cv::String label =
                    cv::String(classNames[objectClass]) + ": " + conf;
                int baseLine = 0;
                cv::Size labelSize = getTextSize(
                    label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

                center.x = center.x - labelSize.width / 2;

                cv::rectangle(
                    frameMat,
                    cv::Rect(
                        cv::Point(center.x, center.y - labelSize.height),
                        cv::Size(labelSize.width, labelSize.height + baseLine)),
                    cv::Scalar(255, 255, 255), cv::FILLED);
                cv::putText(frameMat, label, center, cv::FONT_HERSHEY_SIMPLEX,
                            0.5, cv::Scalar(0, 0, 0));
            }
        }

        /* Display the image */
        cv::imshow("Display Image", frameMat);
    }

    return 0;
}
