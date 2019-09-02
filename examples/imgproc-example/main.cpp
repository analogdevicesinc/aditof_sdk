#include "basic_controller.h"
#include "imgproc_controller.h"

#include <glog/logging.h>
#include <iostream>

using namespace aditof;

int main(int argc, char *argv[]) {

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    auto basic_controller = std::make_shared<Basic_Controller>();
    std::vector<aditof::Frame> frames = basic_controller->captureFrames();
    if (!frames.empty())
        basic_controller->printFrame(frames[0]);
    //  aditof::Frame firstFrame = basic_controller->getFirstFrame();
    // basic_controller->printFrame(firstFrame);
    return 0;
}
