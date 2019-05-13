#if 1

#include "aditofdemocontroller.h"
#include "aditofdemoview.h"
#include <glog/logging.h>
#include <iostream>

int main(int argc, char **argv) {

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    auto controller = std::make_shared<AdiTofDemoController>();
    auto view = std::make_shared<AdiTofDemoView>(controller, "Hello World");
    view->render();

    return 0;
}

#else

#include <camera.h>
#include <frame.h>
#include <glog/logging.h>
#include <iostream>
#include <system.h>

using namespace aditof;

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    System &system = System::instance();
    system.initialize();

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraList(cameras);

    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();
    camera->initialize();

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type avaialble!";
        return 0;
    }
    camera->setFrameType(frameTypes.front());

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        std::cout << "no camera modes available!";
        return 0;
    }
    camera->setMode(modes.front());

    camera->start();

    auto frame = std::make_shared<aditof::Frame>();
    camera->requestFrame(frame);

    camera->stop();

    uint16_t *data1;
    frame->getData(FrameDataType::RAW, &data1);
    if (!data1) {
        std::cout << "no memory allocated in frame";
        return 0;
    }

    FrameDetails fDetails;
    frame->getDetails(fDetails);
    for (unsigned int i = 0; i < fDetails.width * fDetails.height; ++i) {
        std::cout << data1[i] << " ";
    }

    return 0;
}

#endif
