#include <iostream>


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
		// system("clear");
		std::cout << value << std::endl;
		// sleep(1);
	}

	bool isOk()
	{
		if(value < 100){
			return true;
		}
		return false;
	}
};

