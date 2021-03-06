
#include "IDisparity.h"
#include "SGBM.h"
#include "ProcessA.h"
#include "ProcessB.h"
#include "ProcessC.h"
#include "ProcessK.h"
#include "ProcessIPM.h"
#include "ProcessBGS.h"
#include "IDetection.h"
#include "DetectionA.h"
#include "IControl.h"
#include "ICapture.h"
#include "IProcess.h"
#include "Capture.h"

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

//#define sgbm
#define cap
//#define proca // background substractor ktory nerobi nic
//#define procb // jozkov linedetect
#define procc // hladanie ciar patoziak
//#define prock
#define procIPM // inverzne mapovanie
//#define procBGS //chyba, asi nespravna velkost gradient.png
//#define deta //pada do chyby treba opravit


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
	IProcess *processRemoveGradient, *processLaneDetect, *processC, *processK, *processIPM, *processBGS;
	property_tree::ptree pt;  // citac .ini suborov
	cv::Mat frameLeft, frameRight, frameDisparity, frameBirdView, frameLaneDetect, frameRemovedGradient, frameProcessC, frameCascade 
		, frameCloseObj, frameMediumObj, frameFarObj, frameIPM, frameBGS, MatObjectDetected;
	bool durotationThreads;
	int isCameraZed;
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
			processK = new ProcessK();
		#endif	

		#ifdef procIPM
			processIPM = new ProcessIPM();
		#endif

#ifdef procBGS
			processBGS = new ProcessBGS();
#endif

		#ifdef cap
			property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
			string strPath1 = pt.get<string>("VideoInput.VideoInput1");
			string strPath2 = pt.get<string>("VideoInput.VideoInput2");
			int iPath1,iPath2;
			try
			{
				 iPath1 = boost::lexical_cast<int>(strPath1);   // ak je v subore cislo = vstup z kamery
				 iPath2 = boost::lexical_cast<int>(strPath2);

				capture = new Capture(iPath1, iPath2);
			}
			catch (...)
			{
				capture = new Capture(strPath1, strPath2);  // ak je v subore cesta na video file
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

		property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			isCameraZed = boost::lexical_cast<int>(pt.get<string>("VideoInput.CameraZed"));
			
		}
		catch (...)
		{
			cout << "Error in parsing Filter in CaptureZen3D!" << endl;
		}

		namedWindow("Video input 1", cv::WINDOW_AUTOSIZE);
		namedWindow("Video input 2", cv::WINDOW_AUTOSIZE);
		//namedWindow("Disparita", cv::WINDOW_AUTOSIZE);
		//namedWindow("LineAssist", cv::WINDOW_AUTOSIZE);
		//namedWindow("InversePerspectiveMapping", WINDOW_AUTOSIZE);
		cv::moveWindow("Video input 1", 0, 0);
		cv::moveWindow("Video input 2", 490, 0);
		cv::moveWindow("Disparita", 0, 350);
		cv::moveWindow("LineAssist", 490, 350);

		//tu sa budu volat metody a robit hlavny tok
		while (1)//0 ak nechceme aby sa to robilo
		{
			capture->process();
			
			frameLeft = capture->getLeftRGB();
			frameRight = capture->getRightRGB();
			

			if (!frameLeft.empty() && !frameRight.empty())
			{	
				try {
					cv::imshow("Video input 1", frameLeft);
					cv::imshow("Video input 2", frameRight);
				}
				catch(cv::Exception & e){
					cerr << e.msg << endl;
				}

#ifdef sgbm		
				if (isCameraZed == 1) {
					
					cv::Mat frameDisparity1, frameDisparity2;
					frameDisparity = capture->getDepthMap();

				}
				else {

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
	
#ifdef proca
				processRemoveGradient->process(frameDisparity, frameLeft);
				lockThred.lock();
				frameRemovedGradient = processRemoveGradient->getFrame();
				lockThred.unlock();
#endif

#ifdef procIPM
				if (durotationThreads)
					then = system_clock::now();

				processIPM->process(frameLeft, frameRight);
				lockThred.lock();
				frameIPM = processIPM->getFrame();
				lockThred.unlock();

				if (durotationThreads)
				{
					now = system_clock::now();
					cout << "Execution Time InversePerspectiveMapping:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
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
#ifdef procBGS
					if (durotationThreads)
						then = system_clock::now();

					processBGS->process(frameDisparity, frameRight);
					lockThred.lock();
					frameBGS = processBGS->getFrame();
					MatObjectDetected = processBGS->getObject();
					lockThred.unlock();

					if (durotationThreads)
					{
						now = system_clock::now();
						cout << "Execution Time InversePerspectiveMapping:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
					}
					imshow("FrameBGS", frameBGS);
					cout << MatObjectDetected << endl;
#endif
				}
				else { cout << "Frame disparity is Empty!" << endl; }

				

				


#ifdef procb
				if (!frameLaneDetect.empty())
				{
					imshow("LineAssist", frameLaneDetect);
				}
				else { cout << "Frame LineDetect is Empty!" << endl; }
#endif
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

				

				if (!frameIPM.empty())
				{
					imshow("InversePerspectiveMapping", frameIPM);

#ifdef procc
					if (durotationThreads)
						then = system_clock::now();

					processC->process(frameIPM, frameRight);
					lockThred.lock();
					frameProcessC = processC->getFrame();
					lockThred.unlock();

					if (durotationThreads)
					{
						now = system_clock::now();
						cout << "Execution Time InversePerspectiveMapping:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
					}

					if (!frameProcessC.empty())
					{
						imshow("ProcessC", frameProcessC);
					}
					else { cout << "Frame processC is Empty!" << endl; }
#endif

#ifdef prock
					if (durotationThreads)
						then = system_clock::now();

					processK->process(frameLeft, frameLeft);
					lockThred.lock();
					frameCascade = processK->getFrame();
					lockThred.unlock();

					if (durotationThreads)
					{
						now = system_clock::now();
						cout << "Execution Time CASCADES:" << duration_cast<milliseconds>(now - then).count() << " ms" << endl;
					}
					if (!frameCascade.empty())
					{
						imshow("Lane_Cascade", frameCascade);

					}
					else { cout << "Frame Lane_Cascade is Empty!" << endl; }
#endif



				}
				


				
			}
			else { cout << "Frame Left or Right are Empty!" << endl; }
			
			cv::waitKey(1);
			
			char key = (char)cv::waitKey(30);
			switch (key)
			{
			case 'q':
			case 'Q':
			case 's': imwrite("disparita.png",frameDisparity);
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