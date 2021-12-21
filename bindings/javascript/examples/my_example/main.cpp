
#include <iostream>
#include <emscripten/bind.h>

using namespace emscripten;


// using namespace std;

class Counter
{

private:
	int value;

public:

	Counter()
	{
		value = 0;
	}
	void increment()
	{
		value++;
		std::cout << value << std::endl;
	}

	bool isOk()
	{
		if(value < 100){
			return true;
		}
		return false;
	}
};






void callMyName(std::string name){
  std::cout << "Hello " + name + "!"<< std::endl;
}


int main(int argc, char const *argv[]){
  Counter c;
  
  callMyName("world");

  return 0;
}

EMSCRIPTEN_BINDINGS(module) {
    function("callMyName", &callMyName);

    class_<Counter>("Counter")
    .constructor<>()
    .function("increment", &Counter::increment)
    .function("isOk", &Counter::isOk)
    ;
}