#include <aditof/frame.h>
#include <aditof/status_definitions.h>

#include <opencv2/core.hpp>
namespace aditof {
aditof::Status fromFrameToDepthMat(aditof::Frame &frame, cv::Mat &mat) {
    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height) / 2;
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *depthData;
    frame.getData(aditof::FrameDataType::DEPTH, &depthData);

    if (depthData == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    mat = cv::Mat(frameHeight, frameWidth, CV_16UC1, depthData);

    return aditof::Status::OK;
}

aditof::Status fromFrameToIrMat(aditof::Frame &frame, cv::Mat &mat) {
    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height) / 2;
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *irData;
    frame.getData(aditof::FrameDataType::IR, &irData);

    if (irData == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    mat = cv::Mat(frameHeight, frameWidth, CV_16UC1, irData);

    return aditof::Status::OK;
}
} // namespace aditof
