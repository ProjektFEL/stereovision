
#include "IDisparity.h"
#include "DispSGBM.h"
#include "IControl.h"
#include "ICapture.h"
#include "CapZEN3D.h"


#include <stdio.h>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"

using namespace cv;
using namespace std;

class Application{
private:
	//definicia objektov typu interface
	IDisparity *disparity;
	ICapture *capture;
public:
	void init()
	{
		//vytvorenie a inicializacia objektov (smernikov)
		__if_exists(IDisparity)
		{
			disparity = new DispSGBM();
		}

		__if_exists(CapZEN3D)
		{
			capture = new CapZEN3D();
		}
		//tu inicializujes dalsie objekty napr. procCascades ...
	}

	void cycle()
	{
		VideoCapture cap0 = VideoCapture("C:\\Users\\Gamer\\Desktop\\video\\left.mp4");
		VideoCapture cap1 = VideoCapture("C:\\Users\\Gamer\\Desktop\\video\\right.mp4");
		Mat left, right;
		Mat rgb, depth, object;
		//int wheel; //bool direction; int strength, bool brake;
		
		//rgb = capture->getRGB();
		//depth = capture->getDepthMap();
		/*
		cap0->set(CV_CAP_PROP_FRAME_HEIGHT, 320);
		cap0->set(CV_CAP_PROP_FRAME_WIDTH, 240);
		cap0->set(CV_CAP_PROP_FPS, 30);
		cap1->set(CV_CAP_PROP_FRAME_HEIGHT, 320);
		cap1->set(CV_CAP_PROP_FRAME_WIDTH, 240);
		cap1->set(CV_CAP_PROP_FPS, 30);
		*/

		//tu sa budu volat metody a robit hlavny tok
		while (1)
		{
			//capture->process();
			//disparity->calculate();

			disparity->calculate();
			
			if (waitKey(30) >= 0) break;
		}
	}

	void term()
	{
		__if_exists(CapZEN3D)
		{
			capture->~ICapture();
		}

		__if_exists(DispSGBM)
		{
			disparity->~IDisparity();
		}
	}
};