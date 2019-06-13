#include "aditofdemocontroller.h"
#include "aditofdemoview.h"
#include <glog/logging.h>
#include <iostream>

int main(int argc, char **argv) {

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    auto controller = std::make_shared<AdiTofDemoController>();
    auto view =
        std::make_shared<AdiTofDemoView>(controller, "aditof-demo v0.1");
    view->render();

    return 0;
}
