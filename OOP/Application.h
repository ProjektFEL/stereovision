
#include "IDisparity.h"
#include "DispSGBM.h"
#include "ProcessA.h"
#include "ProcessB.h"
#include "ProcessC.h"
#include "ProcessK.h"
#include "IDetection.h"
#include "DetectionA.h"
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
#include <thread>
#include <mutex>
#include <chrono>
#include <ctime>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/video/background_segm.hpp"

#define sgbm
#define capzed3d
#define proca
#define procb
//#define procc
//#define prock
//#define deta

using namespace cv;
using namespace std;
using namespace boost;
using namespace std::chrono;



class Application{
private:
	//vlakna
	thread *t1, *t2, *t3;
	std::mutex lockThred;
	//definicia objektov typu interface
	IDisparity *disparity;
	ICapture *capture;
	IDetection *detectionA;
	IProcess *processRemoveGradient, *processLaneDetect, *processC, *processK;
	property_tree::ptree pt;  // citac .ini suborov
	Mat frameLeft, frameRight, frameDisparity, frameBirdView, frameLaneDetect, frameRemovedGradient, frameProcessC, frameCascade 
		, frameCloseObj, frameMediumObj, frameFarObj;
	bool durotationThreads;
	system_clock::time_point then, now;
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

		#ifdef prock
			processK = new ProcessK(t2);
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
	   
		#ifdef deta
			detectionA = new DetectionA();
		#endif

			durotationThreads = false;
		//tu inicializujes dalsie objekty napr. procCascades ...
	}

	void cycle()
	{	
		//int wheel; //bool direction; int strength, bool brake;

		namedWindow("Video input 1", WINDOW_AUTOSIZE);
		namedWindow("Video input 2", WINDOW_AUTOSIZE);
		namedWindow("Disparita", WINDOW_AUTOSIZE);
		namedWindow("LineAssist", WINDOW_AUTOSIZE);
		moveWindow("Video input 1", 0, 0);
		moveWindow("Video input 2", 490, 0);
		moveWindow("Disparita", 0, 350);
		moveWindow("LineAssist", 490, 350);

		//tu sa budu volat metody a robit hlavny tok
		while (1)//0 ak nechceme aby sa to robilo
		{
			capture->process();
			lockThred.lock();
			frameLeft = capture->getLeftRGB();
			frameRight = capture->getRightRGB();
			lockThred.unlock();

			if (!frameLeft.empty() && !frameRight.empty())
			{			
				imshow("Video input 1", frameLeft);
				imshow("Video input 2", frameRight);

#ifdef sgbm			
				if (durotationThreads)
					then = system_clock::now();

				t1 = disparity->run(&lockThred, frameLeft, frameRight);
				lockThred.lock();
				frameDisparity = disparity->getDisparity();
				lockThred.unlock();

				if (durotationThreads)
				{ 
					now = system_clock::now();
					cout << "Execution Time DISPARITY:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
				}
				
#endif

#ifdef procb
				if (durotationThreads)
					then = system_clock::now();

				t2 = processLaneDetect->run(&lockThred, frameLeft, frameRight);
				lockThred.lock();
				frameLaneDetect = processLaneDetect->getFrame();
				lockThred.unlock();

				if (durotationThreads)
				{
					now = system_clock::now();
					cout << "Execution Time LANE DETECT:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
				}
#endif
				
				processRemoveGradient->process(frameDisparity, frameLeft);
				lockThred.lock();
				frameRemovedGradient = processRemoveGradient->getFrame();
				lockThred.unlock();


#ifdef prock
				if (durotationThreads)
					then = system_clock::now();

				//processK->work(&lockThred, frameLeft, frameRight);
				//lockThred.lock();
				//frameCascade = processK->getFrame();
				//lockThred.unlock();

				if (durotationThreads)
				{
					now = system_clock::now();
					cout << "Execution Time CASCADES:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
				}
#endif

#ifdef deta
				if (durotationThreads)
					then = system_clock::now();

				t3 = detectionA->run(&lockThred, frameDisparity, frameLeft, frameRight);
				lockThred.lock();
				frameCloseObj = detectionA->getCloseObj();
				frameMediumObj = detectionA->getMediumObj();
				frameFarObj = detectionA->getFarObj();
				lockThred.unlock();

				if (durotationThreads)
				{
					now = system_clock::now();
					cout << "Execution Time DETECTION:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
				}
#endif
				
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
				
				if (!frameCloseObj.empty())
				{
					imshow("CloseDistanceObj", frameCloseObj);
				}

				if (!frameMediumObj.empty())
				{
					imshow("MediumDistanceObj", frameMediumObj);
				}

				if (!frameMediumObj.empty())
				{
					imshow("FarDistanceObj", frameFarObj);
				}

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
				term();
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
		}

		if (processLaneDetect)
		{
			delete processLaneDetect;
			processLaneDetect = NULL;
		}

		if (processC)
		{
			delete processC;
			processC = NULL;
		}

		if (capture)
		{
			delete capture;
			capture = NULL;
		}

		if (disparity)
		{
			delete disparity;
			disparity = NULL;
		}

		if (detectionA)
		{
			delete detectionA;
			detectionA = NULL;
		}	

		if (processK)
		{
			delete processK;
			processK = NULL;
		}
	}
};