#include "aditofdemocontroller.h"
#include "aditofdemoview.h"
#include <aditof/version.h>
#include <glog/logging.h>
#include <iostream>

int main(int argc, char **argv) {

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    std::string version = aditof::getApiVersion();
    auto controller = std::make_shared<AdiTofDemoController>();
    auto view =
        std::make_shared<AdiTofDemoView>(controller, "aditof-demo " + version);
    view->render();

    return 0;
}
