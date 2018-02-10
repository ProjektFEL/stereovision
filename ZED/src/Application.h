
#include "IDisparity.h"
#include "DispSGBM.h"
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
#include "CapZED3D.h"
#include "ControlA.h"

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
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <math.h>


#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/video/background_segm.hpp"

#define sgbm
#define capzed3d
//#define proca // background substractor ktory nerobi nic
//#define procb // jozkov linedetect
//#define procc // hladanie ciar patoziak
//#define prock
//#define procIPM // inverzne mapovanie
#define procBGS //chyba, asi nespravna velkost gradient.png
//#define deta //pada do chyby treba opravit
#define controla


using namespace std;
using namespace boost;
using namespace std::chrono;



class Application{
private:
    thread *t1,*t2,*t3;
    std::mutex lockThread;
	//definicia objektov typu interface
	IDisparity *disparity;
	ICapture *capture;
	IDetection *detectionA;
	IProcess *processA, *processLaneDetect, *processC, *processK, *processIPM, *processBGS;
    IControl *controlA;
	property_tree::ptree pt;  // citac .ini suborov
	cv::Mat frameLeft, frameRight, frameDisparity, frameBirdView, frameLaneDetect, frameRemovedGradient, frameProcessC, frameCascade
		, frameCloseObj, frameMediumObj, frameFarObj, frameIPM, frameBGS, MatObjectDetected, backgroundImage;
	bool durotationThreads;
	bool servoControl;
	int uholNatocenia, withMovement, iUholProcessC, iTmpUholProcessC;
	int isCameraZed;
	double uholProcessC;
	cv::Size sizeIMP, sizeImage;


public:

	void init()
	{

		//vytvorenie a inicializacia objektov (smernikov)
		#ifdef sgbm
			disparity = new DispSGBM();
		#endif

		#ifdef proca
			processA = new ProcessA();
		#endif

		#ifdef procb
			processLaneDetect = new ProcessB();
		#endif

		#ifdef procc
			processC = new ProcessC();

			//namedWindow("LaneDetect control C", cv::WINDOW_AUTOSIZE);
            cv::resizeWindow("LaneDetect control C", 400, 700);
		    cv::moveWindow("LaneDetect control C", 1300, 0);
		#endif

		#ifdef prock
			processK = new ProcessK();
		#endif

		#ifdef procIPM
			processIPM = new ProcessIPM();
			//namedWindow("IMP_Control", cv::WINDOW_AUTOSIZE);
			cv::resizeWindow("IMP_Control", 400, 500);
		    cv::moveWindow("IMP_Control", 800, 300);

		#endif

#ifdef procBGS
			processBGS = new ProcessBGS();
#endif

		#ifdef capzed3d
		sizeImage = cv::Size(480, 320);
			property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
			string strPath1 = pt.get<string>("VideoInput.VideoInput1");
			string strPath2 = pt.get<string>("VideoInput.VideoInput2");
			int iPath1,iPath2;
			try
			{
				 cout << "Obraz z kamery" << endl;
				 iPath1 = boost::lexical_cast<int>(strPath1);   // ak je v subore cislo = vstup z kamery
				 iPath2 = boost::lexical_cast<int>(strPath2);

				capture = new CapZED3D(iPath1, iPath2);
			}
			catch (...)
			{
				cout << "nacitavanie zo suboru" << endl;
				capture = new CapZED3D(strPath1, strPath2);  // ak je v subore cesta na video file
			}
		#endif

		#ifdef deta
			detectionA = new DetectionA();
		#endif

		#ifdef controla
			controlA = new ControlA();
			namedWindow("Kolesa", cv::WINDOW_AUTOSIZE);
			cv::moveWindow("Kolesa", 680, 450);
			uholNatocenia = 30;
			withMovement = 0;
			cvCreateTrackbar("Uhol", "Kolesa", &uholNatocenia, 60);
			cvCreateTrackbar("Move", "Kolesa", &withMovement, 1);
		#endif

			durotationThreads = false;
		//tu inicializujes dalsie objekty napr. procCascades ...

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
		namedWindow("Disparita", cv::WINDOW_AUTOSIZE);
		//namedWindow("LineAssist", cv::WINDOW_AUTOSIZE);
		//namedWindow("InversePerspectiveMapping", WINDOW_AUTOSIZE);
		cv::moveWindow("Video input 1", 0, 0);
		cv::moveWindow("Video input 2", 500, 0);
		cv::moveWindow("Disparita", 0, 450);
		//cv::moveWindow("LineAssist", 490, 350);
		servoControl = false;
	}

	void cycle()
	{
		//int wheel; //bool direction; int strength, bool brake;

		//tu sa budu volat metody a robit hlavny tok
		while (1)//0 ak nechceme aby sa to robilo
		{
			//cout << "Before process" << endl;
			capture->process();
			//cout << "After process" << endl;

			//capture->getLeftRGB().copyTo(frameLeft);
            //capture->getRightRGB().copyTo(frameRight);
            if (isCameraZed == 1){
            cv::resize(capture->getLeftRGB(),frameLeft,sizeImage);
            cv::resize(capture->getRightRGB(),frameRight,sizeImage);
            }
            else
            {
            frameLeft = capture->getLeftRGB();
            frameRight = capture->getRightRGB();
            }
            //cout << "Get Left and Right image" << endl;

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
					 capture->getDepthMap().copyTo(frameDisparity);
					 //cout << "Get Depth" << endl;
				}
				else {
/*
					t1 = disparity->run(&lockThred, frameLeft, frameRight);
					lockThred.try_lock();
					frameDisparity = disparity->getDisparity();
					lockThred.unlock();*/
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
				//backgroundImage=cv::imread("gradient.png",CV_LOAD_IMAGE_GRAYSCALE);
				if(!backgroundImage.empty())
				{
				processA->process(frameDisparity, backgroundImage);
				//lockThred.lock();
				frameRemovedGradient = processA->getFrame();
				//lockThred.unlock();
				if (!frameRemovedGradient.empty())
				{
					imshow("ProcesA", frameRemovedGradient);
				}
				}

#endif

#ifdef procIPM

				processIPM->process(frameLeft, frameRight);

				//processIPM->getFrame().copyTo(frameIPM);
				//lockThread.try_lock();
                frameIPM = processIPM->getFrame();
                //lockThread.unlock();

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
					if(!backgroundImage.empty()){
					processBGS->process(frameDisparity, backgroundImage);
					frameBGS = processBGS->getFrame();
					MatObjectDetected = processBGS->getObject();

					imshow("FrameBGS", frameBGS);
					cout << MatObjectDetected << endl;
					}
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
					//string tyDepth = type2str(frameCloseObj.type());
					//std::printf("Left: %s %dx%d \n", tyDepth.c_str(), frameCloseObj.cols, frameCloseObj.rows);

					//cout << frameCloseObj << endl;
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





					if (!frameProcessC.empty())
					{
						imshow("ProcessC", frameProcessC);
					}
					else { cout << "Frame processC is Empty!" << endl; }
				if (!frameIPM.empty())
				{
					imshow("InversePerspectiveMapping", frameIPM);

#ifdef procc
                    processC->process(frameLeft, frameRight);
                    if(servoControl)
                    {
                          uholProcessC = processC->getAngle();
                          if(uholProcessC != 0){
                          if(uholProcessC < -30)
                          {
                          uholProcessC = -30;
                          }
                          else if(uholProcessC > 30)
                          {
                          uholProcessC = 30;
                          }
                          uholProcessC= uholProcessC +30;
                          int iUholProcessC = (int)round(uholProcessC);
                          if(iUholProcessC != iTmpUholProcessC)
                          {
                            controlA->process(iUholProcessC,false);
                            //cout << "Uhol:" << iUholProcessC << endl;
                          }
                          iTmpUholProcessC = iUholProcessC; // pamatanie


                          }


                    }
					//lockThread.try_lock();
					frameProcessC = processC->getFrame();
					//lockThread.unlock();


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

			//cv::waitKey(1);
			//printf("asdf  ");
			//cout << "Check klavesnica" << endl;

			char key = (char)cv::waitKey(5);
			//cout << "waitkey" << endl;
			switch (key)
			{

                        case 'o': {printf("skusobny vypis");
                                  break;}
                        case 'k': {if(servoControl)
                        {
                        servoControl = false;
                        cout << "Servo ovladanie OFF" << endl;
                        }
                        else{
                        servoControl = true;
                        cout << "Servo ovladanie ON" << endl;
                        }
                                  break;}
                        case 'w': {if(withMovement == 0 && servoControl== false)
                                  {controlA->process(uholNatocenia,false);}
                                  else if(withMovement == 1 && servoControl== false)
                                  {controlA->process(uholNatocenia,true);}
                                  break;}
                        case 'a': {if(uholNatocenia >= 3)
                                  {uholNatocenia = uholNatocenia -3;
                                  }
                                  controlA->process(uholNatocenia,false);
                                  break;}
                        case 'd': {if(uholNatocenia <= 57)
                                  {uholNatocenia= uholNatocenia +3;
                                  }
                                  controlA->process(uholNatocenia,false);
                                  break;}
			case 'i': {imwrite("disparita.png",frameDisparity);
                                  break;}

            case 'b':
            case 'B':{
                cv::imwrite("gradient.png",frameDisparity);
                frameDisparity.copyTo(backgroundImage);
                break;}

			case 'q':
			case 'Q':
			case  27: //escape key
				term();
				exit(0);
				break;
			}
			//cout << "Koniec cyklu" << endl;
		}
	}

	void term()
	{
		if (processA)
		{
			delete processA;
			processA = NULL;
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

	string type2str(int type) {
		string r;

		uchar depth = type & CV_MAT_DEPTH_MASK;
		uchar chans = 1 + (type >> CV_CN_SHIFT);

		switch (depth) {
		case CV_8U:  r = "8U"; break;
		case CV_8S:  r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:     r = "User"; break;
		}

		r += "C";
		r += (chans + '0');

		return r;
	}


};
