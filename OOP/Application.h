
#include "IDisparity.h"
#include "DispSGBM.h"
#include "ProcessA.h"
#include "ProcessB.h"
#include "IControl.h"
#include "ICapture.h"
#include "IProcess.h"
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
	IProcess *processBirdview, *processLaneDetect;
	property_tree::ptree pt;  // citac .ini suborov
public:
	void init()
	{
		//vytvorenie a inicializacia objektov (smernikov)
		__if_exists(IDisparity)
		{
			disparity = new DispSGBM();
		}

		__if_exists(IProcess)
		{
			processLaneDetect = new ProcessB();
		}

		__if_exists(CapZEN3D)
		{
			property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
			string strPath1 = pt.get<string>("VideoInput.VideoInput1");
			string strPath2 = pt.get<string>("VideoInput.VideoInput2");
			int iPath1,iPath2;
			try
			{
				 iPath1 = boost::lexical_cast<int>(strPath1);   // ak je v subore cislo = vstup z kamery
				 iPath2 = boost::lexical_cast<int>(strPath2);

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

		namedWindow("Video input 1", WINDOW_AUTOSIZE);
		namedWindow("Video input 2", WINDOW_AUTOSIZE);
		namedWindow("Disparita", WINDOW_AUTOSIZE);
		namedWindow("LineAssist", WINDOW_AUTOSIZE);
		moveWindow("Video input 1", 0, 0);
		moveWindow("Video input 2", 490, 0);
		moveWindow("Disparita", 0, 350);
		moveWindow("LineAssist", 490, 350);

		//tu sa budu volat metody a robit hlavny tok
		while (1)
		{
			capture->process();
			Mat frameLeft, frameRight, frameDisparity, frameBirdView, frameLaneDetect;
			frameLeft = capture->getLeftRGB();
			frameRight = capture->getRightRGB();
			/*processBirdview->process(frameLeft, frameRight);
			frameBirdView = processBirdview->getFrame();*/
			
			processLaneDetect->process(frameLeft, frameRight);
			frameLaneDetect = processLaneDetect->getFrame();
			
			if (!frameLeft.empty() && !frameRight.empty())
			{
				imshow("Video input 1", frameLeft);
				imshow("Video input 2", frameRight);
				disparity->calculate(frameLeft, frameRight);
				frameDisparity = disparity->getDisparity();

				if (!frameDisparity.empty())
				{
					imshow("Disparita", frameDisparity);
				}
				else { cout << "Frame disparity is Empty!" << endl; }

				if (!frameLaneDetect.empty())
				{
					imshow("LineAssist", frameLaneDetect);
				}
				else { cout << "Frame LineDetect is Empty!" << endl; }
			}
			else { cout << "Frame Left or Right are Empty!" << endl; }

			waitKey(1);

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