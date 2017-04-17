
#include "IDisparity.h"
#include "DispSGBM.h"
#include "ProcessA.h"
#include "ProcessB.h"
#include "ProcessC.h"
#include "ProcessK.h"
#include "IControl.h"
#include "ICapture.h"
#include "IProcess.h"
#include "CapZED3D.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

#include <stdio.h>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/video/background_segm.hpp"

#define sgbm
#define capzed3d
#define proca
#define procb
#define procc
#define prock

using namespace cv;
using namespace std;
using namespace boost;




class Application{
private:
	//definicia objektov typu interface
	IDisparity *disparity;
	ICapture *capture;
	IProcess *processRemoveGradient, *processLaneDetect, *processC, *processK;
	property_tree::ptree pt;  // citac .ini suborov
	Mat frameLeft, frameRight, frameDisparity, frameBirdView, frameLaneDetect, frameRemovedGradient, frameProcessC, frameCascade;

	Ptr<BackgroundSubtractor> pMOG; //MOG Background subtractor
	Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
public:
	void init()
	{
		//vytvorenie a inicializacia objektov (smernikov)
		#ifdef sgbm
			disparity = new DispSGBM();
		#endif

		#ifdef proca
			processRemoveGradient = new ProcessA();
		#endif

		#ifdef procb
			processLaneDetect = new ProcessB();
		#endif

		#ifdef procc
			processC = new ProcessC();
		#endif

		#ifdef procc
			processK = new ProcessK();
		#endif

		#ifdef capzed3d
			property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
			string strPath1 = pt.get<string>("VideoInput.VideoInput1");
			string strPath2 = pt.get<string>("VideoInput.VideoInput2");
			int iPath1,iPath2;
			try
			{
				 iPath1 = boost::lexical_cast<int>(strPath1);   // ak je v subore cislo = vstup z kamery
				 iPath2 = boost::lexical_cast<int>(strPath2);

				capture = new CapZED3D(iPath1, iPath2);
			}
			catch (...)
			{
				capture = new CapZED3D(strPath1, strPath2);  // ak je v subore cesta na video file
			}   
		#endif


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

			if (!frameLeft.empty() && !frameRight.empty())
			{			
				imshow("Video input 1", frameLeft);
				imshow("Video input 2", frameRight);

				/*processBirdview->process(frameLeft, frameRight);
				frameBirdView = processBirdview->getFrame();*/

				processLaneDetect->process(frameLeft, frameRight);
				frameLaneDetect = processLaneDetect->getFrame();

				disparity->calculate(frameLeft, frameRight);
				frameDisparity = disparity->getDisparity();

				/*processRemoveGradient->process(frameDisparity, frameLeft);
				frameRemovedGradient = processRemoveGradient->getFrame();*/

				processK->process(frameLeft, frameRight);
				frameCascade = processK->getFrame();

				if (!frameDisparity.empty())
				{
					imshow("Disparita", frameDisparity);
				}
				else { cout << "Frame disparity is Empty!" << endl; }

				if (!frameProcessC.empty())
				{
					imshow("ProcessC", frameProcessC);
				}
				else { cout << "Frame processC is Empty!" << endl; }

				if (!frameLaneDetect.empty())
				{
					imshow("LineAssist", frameLaneDetect);
				}
				else { cout << "Frame LineDetect is Empty!" << endl; }
				
				if (!frameCascade.empty())
				{
					imshow("Lane_Cascade", frameCascade);
				}
				else { cout << "Frame Lane_Cascade is Empty!" << endl; }
			}
			else { cout << "Frame Left or Right are Empty!" << endl; }

			waitKey(1);

			
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
		if (processRemoveGradient)
		{
			delete processRemoveGradient;
			processRemoveGradient = NULL;
			processRemoveGradient->~IProcess();
		}

		if (processLaneDetect)
		{
			delete processLaneDetect;
			processLaneDetect = NULL;
			processLaneDetect->~IProcess();
		}

		if (processC)
		{
			delete processC;
			processC = NULL;
			processC->~IProcess();
		}

		if (capture)
		{
			delete capture;
			capture = NULL;
			capture->~ICapture();
		}

		if (disparity)
		{
			delete disparity;
			disparity = NULL;
			disparity->~IDisparity();
		}
	}
};