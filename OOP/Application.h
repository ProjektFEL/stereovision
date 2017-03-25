
#include "IDisparity.h"
#include "DispSGBM.h"
#include "IControl.h"
#include "ICapture.h"
#include "CapZEN3D.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

#include <stdio.h>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>


using namespace cv;
using namespace std;
using namespace boost;

class Application{
private:
	//definicia objektov typu interface
	IDisparity *disparity;
	ICapture *capture;
	property_tree::ptree pt;  // citac .ini suborov
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
			property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
			string strPath1 = pt.get<string>("VideoInput.VideoInput1");
			string strPath2 = pt.get<string>("VideoInput.VideoInput2");
			int iPath1,iPath2;
			try
			{
				int iPath1 = boost::lexical_cast<int>(strPath1);   // ak je v subore cislo
				int iPath2 = boost::lexical_cast<int>(strPath2);

				capture = new CapZEN3D(iPath1, iPath2);
			}
			catch (...)
			{
				capture = new CapZEN3D(strPath1, strPath2);  // ak je v subore cesta na video file
			}
			   
		}
		//tu inicializujes dalsie objekty napr. procCascades ...
	}

	void cycle()
	{
		
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
			capture->process();
			Mat frameLeft, frameRight, frameDisparity;
			frameLeft = capture->getLeftRGB();
			frameRight = capture->getRightRGB();
			if (!frameLeft.empty() && !frameRight.empty()){
				disparity->calculate(frameLeft, frameRight);
				frameDisparity = disparity->getDisparity();
				imshow("video 1", frameLeft);
				imshow("video 2", frameRight);
				imshow("disparita", frameDisparity);
			}
			waitKey(1);



			//disparity->calculate();
			
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