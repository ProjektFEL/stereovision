#include "Application.h"

using namespace cv;
using namespace std;

int main()
{
	//hlavna aplikacia :) pripadne staci init a cycle a do cycle dat term na spravne miesto...
	/*Application *app = new Application();
	app->init();
	app->cycle();
	app->term();*/
	VideoCapture cap0;
	int camOpen = cap0.open("C:\\Users\\Gamer\\Desktop\\video\\left.mp4");
	while(true) {
		Mat image;
		cap0 >> image;

		if (!image.empty()){
			imshow("window", image);
		}

		// delay 33ms
		waitKey(33);
	}
	return 0;
}