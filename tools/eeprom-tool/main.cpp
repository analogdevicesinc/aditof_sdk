#include <iostream>
#include <stdio.h>
#include <glog/logging.h>

#include "eepromtoolcontroller.h"

int main(){
    auto controller = std::make_shared<EepromToolController>();
    std::cout << controller->hasCamera();

  return 0;
}
