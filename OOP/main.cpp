#include "Application.h"

using namespace cv;
using namespace std;

int main()
{
	//hlavna aplikacia :) pripadne staci init a cycle a do cycle dat term na spravne miesto...
	Application *app = new Application();
	app->init();
	app->cycle();
	app->term();
	
	
	return 0;
}