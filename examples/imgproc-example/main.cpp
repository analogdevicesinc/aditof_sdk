#include "basic_controller.h"
#include "basic_gui.h"
#include "imgproc_controller.h"

#include <glog/logging.h>
#include <iostream>

int main(int argc, char *argv[]) {

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    auto basic_controller = std::make_shared<Basic_Controller>();
    auto basic_gui = std::make_shared<Basic_GUI>("version1");
    basic_gui->renderOnce();
    bool stopFlag = 1;
    while (stopFlag) {
        basic_gui->setDistanceValue(basic_controller->getRange());
        aditof::Frame frame = basic_controller->getFrame();
        // basic_controller->printFrame(frame);
        basic_gui->setFrame(frame);
        stopFlag = basic_gui->renderCyclic();
    }

    return 0;
}
