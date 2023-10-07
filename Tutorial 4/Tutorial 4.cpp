#include <iostream>
#include "VisualDebugger.h"
#include <thread>

using namespace std;

int main()
{
	try 
	{ 
		VisualDebugger::Init("Tutorial 4", 800, 800); 
	}
	catch (Exception exc) 
	{ 
		cerr << exc.what() << endl;
		return 0; 
	}

	VisualDebugger::StartRender ();

	return 0;
}