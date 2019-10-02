#include "basic_controller.h"
#include "basic_gui.h"

#include <glog/logging.h>
#include <iostream>

int main(int argc, char *argv[]) {

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    auto basic_controller = std::make_shared<Basic_Controller>();
    auto basic_gui = std::make_shared<Basic_GUI>("frame-filters");
    basic_gui->initRendering();
    bool stopFlag = 1;
    basic_gui->setRange(basic_controller->getRange());
    while (stopFlag) {
        aditof::Frame frame = basic_controller->getFrame();
        basic_gui->setFrame(frame);
        stopFlag = basic_gui->renderCyclic();
        int mode = basic_gui->getMode();
        if (mode != 3) {
            basic_controller->setMode(mode);
            basic_gui->setRange(basic_controller->getRange());
        }
    }

    return 0;
}
