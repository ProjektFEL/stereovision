#include "Application.h"

using namespace cv;
using namespace std;

int main()
{
	Application *app = new Application();
	app->init();
	app->cycle();
	app->term();
	
#ifdef _DEBUG
	system("pause");
#endif
	
	return 0;
}