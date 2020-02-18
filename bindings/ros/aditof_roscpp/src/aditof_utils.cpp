#include "aditof_utils.h"

#include <aditof/camera_96tof1_specifics.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>
#include <ros/ros.h>

using namespace aditof;
std::shared_ptr<Camera> initCameraEthernet(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
    Status status = Status::OK;

    if (argc < 2) {
        LOG(ERROR) << "No ip provided!";
        return nullptr;
    }

    std::string ip = argv[1];

    System system;
    status = system.initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize system!";
        return nullptr;
    }

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraListAtIp(cameras, ip);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return nullptr;
    }

    std::shared_ptr<Camera> camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return nullptr;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type avaialble!";
        return nullptr;
    }

    status = camera->setFrameType(frameTypes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return nullptr;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return nullptr;
    }

    status = camera->setMode(modes[1]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return nullptr;
    }

    return camera;
}

void applyNoiseReduction(const std::shared_ptr<Camera> &camera, int argc,
                         char **argv) {

    // TODO add support for parameter dynamic reconfiguration
    if (argc < 3) {
        LOG(ERROR) << "No threshold value provided!";
        return;
    }

    int threshold = std::stoi(argv[2]);
    auto specifics = camera->getSpecifics();
    auto cam96tof1Specifics =
        std::dynamic_pointer_cast<Camera96Tof1Specifics>(specifics);
    if (cam96tof1Specifics) {
        cam96tof1Specifics->setNoiseReductionThreshold(threshold);
        cam96tof1Specifics->enableNoiseReduction(true);
    }
}

uint16_t *getNewFrame(const std::shared_ptr<Camera> &camera,
                      aditof::Frame *frame) {
    Status status = Status::OK;
    status = camera->requestFrame(frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return nullptr;
    }

    uint16_t *frameData;
    status = frame->getData(FrameDataType::RAW, &frameData);

    if (status != Status::OK) {
        LOG(ERROR) << "Could not get frame data!";
        return nullptr;
    }

    if (!frameData) {
        LOG(ERROR) << "no memory allocated in frame";
        return nullptr;
    }
    return frameData;
}

IntrinsicParameters getIntrinsics(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.intrinsics;
}

int getRangeMax(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.maxDepth;
}

int getRangeMin(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.minDepth;
}
