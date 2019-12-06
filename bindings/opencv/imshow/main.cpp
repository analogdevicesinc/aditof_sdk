#include <aditof/camera.h>
#include <aditof/device_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../aditof_opencv.h"

using namespace aditof;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    Status status = Status::OK;

    System system;
    status = system.initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize system!";
        return 0;
    }

    std::vector<std::shared_ptr<Camera>> cameras;
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

    status = camera->setMode(modes[0]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    int cameraRange = cameraDetails.range;
    aditof::Frame frame;

    const size_t REGS_CNT = 5;
    uint16_t afeRegsAddr[REGS_CNT] = {0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22};
    uint16_t afeRegsVal[REGS_CNT] = {0x0006, 0x0004, 0x803C, 0x0007, 0x0004};

    auto device = camera->getDevice();
    aditof::Status registerAFEwriting =
        device->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);

    while (cv::waitKey(1) != 27 &&
           getWindowProperty("Display Image", cv::WND_PROP_AUTOSIZE) >= 0) {

        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }

        /* Convert from frame to depth mat */
        cv::Mat mat;
        status = fromFrameToDepthMat(frame, mat);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return 0;
        }

        /* Distance factor */
        double distance_scale = 255.0 / cameraRange;

        /* Convert from raw values to values that opencv can understand */
        mat.convertTo(mat, CV_8U, distance_scale);

        /* Apply a rainbow color map to the mat to better visualize the
         * depth data */
        applyColorMap(mat, mat, cv::COLORMAP_RAINBOW);

        /* Display the image */
        imshow("Display Image", mat);
    }

    return 0;
}
