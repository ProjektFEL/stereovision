
#include "IDisparity.h"
#include "DispSGBM.h"
#include "ProcessA.h"
#include "ProcessB.h"
#include "ProcessC.h"
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
#include "opencv2/video/background_segm.hpp"


using namespace cv;
using namespace std;
using namespace boost;

class Application{
private:
	//definicia objektov typu interface
	IDisparity *disparity;
	ICapture *capture;
	IProcess *processRemoveGradient, *processLaneDetect, *processC;
	property_tree::ptree pt;  // citac .ini suborov
	Mat frameLeft, frameRight, frameDisparity, frameBirdView, frameLaneDetect, frameRemovedGradient, frameProcessC;

	Ptr<BackgroundSubtractor> pMOG; //MOG Background subtractor
	Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
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
		__if_exists(IProcess)
		{
			processRemoveGradient = new ProcessA();
		}
		__if_exists(IProcess)
		{
			processC = new ProcessC();
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


		pMOG = createBackgroundSubtractorKNN(); //MOG approach
		pMOG2 = createBackgroundSubtractorMOG2();
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
				/*processRemoveGradient->process(frameDisparity, frameLeft);
				frameRemovedGradient = processRemoveGradient->getFrame();*/

				/*processC->process(frameLeft, frameRight);
				frameProcessC= processC->getFrame();                      pTo v processC som skusal hladat ciary*/

				if (!frameProcessC.empty())
				{
					imshow("ProcessC", frameProcessC);
				}

				if (!frameDisparity.empty())
				{
					imshow("Disparita", frameDisparity);
				}
				else { cout << "Frame disparity is Empty!" << endl; }

				Mat fgMaskMOG; //fg mask generated by MOG method
				Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
				
				Mat framebgsub1, framebgsub2,video1,video2,hhs;
				
				pMOG->apply(frameDisparity, framebgsub1);
				pMOG2->apply(frameDisparity, framebgsub2);

				/*frameDisparity.copyTo(hhs, framebgsub1);*/

				//hconcat(frameDisparity, framebgsub1, video1);
				////hconcat(frameLeft, video1, video2);
				//hconcat(video1, hhs, video2);

				if (!frameLaneDetect.empty())
				{
					imshow("LineAssist", frameLaneDetect);
				}
				else { cout << "Frame LineDetect is Empty!" << endl; }
				if (!framebgsub1.empty())
				{
					imshow("RemovedGradient", framebgsub1);
				}
				else { cout << "Frame RemovedGradient is Empty!" << endl; }
			}
			else { cout << "Frame Left or Right are Empty!" << endl; }

			waitKey(1);

			if (waitKey(30) >= 0) break;
			char key = (char)waitKey(30);
			switch (key)
			{
			case 'q':
			case 'Q':
			case  27: //escape key

				// cvReleaseVideoWriter(CvVideoWriter** writer)
				exit(0);
				break;
			}
		
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