

#include "main.h"
#include <emscripten/bind.h>

using namespace emscripten;

void callMyName(std::string name){
  std::cout << "Hello " + name + "!"<< std::endl;
}


int main(int argc, char const *argv[]){
  Counter c;
  
  callMyName("world");
  // while(c.isOk()){
  // // while(true) {
  //   c.increment();
  // }

  return 0;
}

EMSCRIPTEN_BINDINGS(module) {
    function("callMyName", &callMyName);

    class_<Counter>("Counter")
    .constructor<>()
    .function("increment", &Counter::increment)
    .function("isOk", &Counter::isOk)
    // .property("value", &Counter::value)
    ;
}