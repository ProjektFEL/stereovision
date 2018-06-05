
#include "IDisparity.h"
#include "DispSGBM.h"
#include "ProcessA.h"
#include "ProcessB.h"
#include "ProcessC.h"
#include "ProcessK.h"
#include "ProcessS.h"
#include "ProcessPython.h"
#include "ProcessIPM.h"
#include "ProcessBGS.h"
#include "IDetection.h"
#include "DetectionA.h"
#include "IControl.h"
#include "ICapture.h"
#include "IProcess.h"
#include "CapZED3D.h"
#include "ControlA.h"
#include "ControlB.h"
#include "ControlUI.h"

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
#include <iomanip>
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
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


#define sgbm       // disparita a jej vypocet OpenCV ak bol pouzity vlastny build kamier
#define capzed3d   // hlavna trieda na ziskavanie obrazu z kamier + disparitu
//#define proca   // background substractor ktory nerobi nic
//#define procb   // jozkov linedetect (o nicom :P)
//#define procc   // hladanie ciar patoziak (fungujuci)
//#define prock   // kaskady na hladanie ciar (v configu sa zadava nazov)
//#define procIPM   // inverzne mapovanie (je obrazovym vstupom pre procC na prilizenie na cestu)
//#define procBGS   // process na odstranenia pozadie na zaklade diferenciacii od statickeho pozadia odcitanim
#define procs       // navigacia disparitou, hladanie volnej cesty
//#define procPipe   // ulozenie obrazka do pipe (to malo byt na prepojenie s neuronkou)
//#define deta       // detekcia prekazok a nasledna navigacia, pada do chyby treba opravit (stary koncept)
//#define controla   // pomocou ciar, lane assist dava uhol natocenia tak tento control posle uhol dalej motoru
#define controlb   // pomocou vyhybania prekazkam, na vstupe ma MAT s poziciami prekazok a na zaklade toho control posle uhol natocenia motoru
//#define controlUInt // control na navigaciu pomocou ciar a detekciu prekazok (staci iba control mat odkomentovany)

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
	IProcess *processA, *processLaneDetect, *processC, *processK, *processIPM, *processBGS, *processS, *processPython;
    IControl *controlA, *controlB, *controlUI;
	property_tree::ptree pt;  // citac .ini suborov
	cv::Mat frameLeft, frameRight, frameDisparity, frameBirdView, frameLaneDetect, frameRemovedGradient, frameProcessC, frameProcessCThresh, frameCascade
		, frameCloseObj, frameMediumObj, frameFarObj, frameIPM, frameBGS, MatObjectDetected, backgroundImage, frameprocessSBlizke, frameprocessSStredne,
		frameLeftFPS;
	bool durotationThreads;
	bool servoControl, isControlB, movementProcessS, boolbrake, isVideoCapture, isControlUI, isFrameCapture;
	int uholNatocenia, withMovement, iUholProcessC, iTmpUholProcessC;
	int isCameraZed;
	double uholProcessC;
	cv::Size sizeIMP, sizeImage;
	cv::VideoWriter outputFrameLeft;
	/*fps variables*/
	long frameCounter;
	std::time_t timeBegin, timeNow, rawtime;
	struct tm * timeinfo;
        string strPathFrameLeft, strPathFrameRight, strPathFrameDisparity, strPathFrameIPM, strPathFrameProcCLane, strPathFrameProcCThresh, strPathFrameProcSBlizke, strPathFrameProcSStredne;
        short bitSaveFrameLeft, bitSaveFrameRight,bitSaveFrameIPM, bitSaveFrameDisparity, bitSaveFrameProcC, bitSaveFrameProcS;
	int tick, tick2;
	string napisFPS, placeHolder1, placeholdbbessdssemree;


public:

	void init()
	{
        boolbrake = false;
        isVideoCapture = false;
        isFrameCapture = false;



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


                    //cv::resizeWindow("LaneDetect control C", 400, 700);
		    //cv::moveWindow("LaneDetect control C", 1300, 0);
		#endif

		#ifdef prock
			processK = new ProcessK();
		#endif

		#ifdef procPipe
			processPython = new ProcessPython();
		#endif

		#ifdef procs
			processS = new ProcessS();
			movementProcessS = false;
		#endif

		#ifdef procIPM

			processIPM = new ProcessIPM();

			//cv::resizeWindow("IMP_Control", 400, 500);
		        //cv::moveWindow("IMP_Control", 800, 300);
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

		#ifdef controlb
			controlB = new ControlB();
			withMovement = 0;
			isControlB = false;
		#endif

		#ifdef controlUInt

			withMovement = 0;
			uholNatocenia = 30;
			processS = new ProcessS();
			processC = new ProcessC();
			controlUI = new ControlUI();

		#endif

			durotationThreads = false;
		//tu inicializujes dalsie objekty napr. procCascades ...

			property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			isCameraZed = boost::lexical_cast<int>(pt.get<string>("VideoInput.CameraZed"));
                        //nacitanie z config parametre ukladania obrazkov
                        bitSaveFrameLeft = boost::lexical_cast<int>(pt.get<string>("Saved_Frames.bitSaveFrameLeft"));
                        strPathFrameLeft = pt.get<string>("Saved_Frames.pathFrameLeft");
                        bitSaveFrameRight = boost::lexical_cast<int>(pt.get<string>("Saved_Frames.bitSaveFrameRight"));
                        strPathFrameRight = pt.get<string>("Saved_Frames.pathFrameRight");
                        bitSaveFrameDisparity = boost::lexical_cast<int>(pt.get<string>("Saved_Frames.bitSaveFrameDisparity"));
                        strPathFrameDisparity = pt.get<string>("Saved_Frames.pathFrameDisparity");
                        bitSaveFrameIPM = boost::lexical_cast<int>(pt.get<string>("Saved_Frames.bitSaveFrameIPM"));
                        strPathFrameIPM = pt.get<string>("Saved_Frames.pathFrameIPM");
                        bitSaveFrameProcC = boost::lexical_cast<int>(pt.get<string>("Saved_Frames.bitSaveFrameProcC"));
                        strPathFrameProcCLane = pt.get<string>("Saved_Frames.pathFrameProcCLane");
                        strPathFrameProcCThresh = pt.get<string>("Saved_Frames.pathFrameProcCThresh");
                        bitSaveFrameProcS = boost::lexical_cast<int>(pt.get<string>("Saved_Frames.bitSaveFrameProcS"));
                        strPathFrameProcSBlizke = pt.get<string>("Saved_Frames.pathFrameProcSBlizke");
                        strPathFrameProcSStredne = pt.get<string>("Saved_Frames.pathFrameProcSStredne");

		}
		catch (...)
		{
			cout << "Error in parsing Filter in Application!" << endl;
		}

        cout << "pred oknom" << endl;
		namedWindow("FrameLeft", cv::WINDOW_AUTOSIZE);
		//namedWindow("FrameRight", cv::WINDOW_AUTOSIZE);
		namedWindow("Disparita", cv::WINDOW_AUTOSIZE);
		//namedWindow("LineAssist", cv::WINDOW_AUTOSIZE);
		//namedWindow("InversePerspectiveMapping", WINDOW_AUTOSIZE);
		cv::moveWindow("FrameLeft", 0, 0);
		//cv::moveWindow("FrameRight", 500, 0);
		cv::moveWindow("Disparita", 0, 450);
		//cv::moveWindow("LineAssist", 490, 350);
		servoControl = false;
		isControlUI = false;
		cout << "Koniec init" << endl;



        /*fps variables*/
	     frameCounter = 0;


	}

	void cycle()
	{
		//int wheel; //bool direction; int strength, bool brake;
        /*fps variables*/
        timeBegin = std::time(0);
        tick = 0;
        int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
        outputFrameLeft = cv::VideoWriter("videoout.avi", fourcc, 10, cv::Size(480,320), true);
//outputFrameLeft.open("videoout.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(320,480), true);
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

					frameCounter++;
					timeNow = std::time(0) - timeBegin;
					if(timeNow - tick >= 1)
					{
					tick++;
					napisFPS = to_string(frameCounter);
					frameCounter = 0;
					}
					frameLeft.copyTo(frameLeftFPS);
					cv::putText(frameLeftFPS,napisFPS,cvPoint(0,25),cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(255,255,255), 2);

					if(isVideoCapture)
					{

					//outputFrameLeft.open("frameleft",CV_FOURCC('M', 'J', 'P', 'G'),15,frameLeftFPS.size(),true);
					if(outputFrameLeft.isOpened())
					{
					cout << "nahravanie je otvorene" << endl;
					frameLeftFPS.convertTo(frameLeftFPS, CV_8UC1);
					outputFrameLeft.write(frameLeftFPS);
					//outputFrameLeft << frameLeftFPS;
					}

					}
					if(isFrameCapture)
					{
                                            if(bitSaveFrameLeft == 1)
                                              {
                    milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
                    std::string str = to_string(ms.count());
                    std::string meno = str + ".jpeg";
                    cv::imwrite(strPathFrameLeft+ meno,frameLeftFPS);
                    }
                    if(bitSaveFrameRight == 1)
                                              {
                    milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
                    std::string str = to_string(ms.count());
                    std::string meno = str + ".jpeg";
                    cv::imwrite(strPathFrameRight+ meno,frameRight);}
					}
					#ifdef procPipe
					//t3 = processPython->run(&lockThread, frameLeft, frameRight);

					processPython->process(frameLeft, frameRight);

					#endif // procPipe

					cv::imshow("FrameLeft", frameLeftFPS);
					//cv::imshow("FrameRight", frameRight);

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

#ifdef procs
                //cout << "zaciatok procS " << endl;
                if(!frameDisparity.empty()){
				processS->process(frameDisparity, frameRight);

				//processIPM->getFrame().copyTo(frameIPM);
				//lockThread.try_lock();
                frameprocessSBlizke = processS->getFrame();
                frameprocessSStredne = processS->getDisparity();
                //lockThread.unlock();
                if(!frameprocessSBlizke.empty() && !frameprocessSStredne.empty())
                {
                MatObjectDetected = processS->getObject();
                if(isFrameCapture)
					{
                                           if(bitSaveFrameProcS == 1 && !MatObjectDetected.empty()){
                                              milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
                                              std::string str = to_string(ms.count());
                                              std::string menoB = "B"+str + ".jpeg";
                                              std::string menoS = "S"+str + ".jpeg";
                                              cv::imwrite(strPathFrameProcSBlizke+ menoB,frameprocessSBlizke);
                                              cv::imwrite(strPathFrameProcSStredne+ menoS,frameprocessSStredne);}

					}
                cv::imshow("DisparitaBlizke",frameprocessSBlizke);
                cv::imshow("DisparitaStredne",frameprocessSStredne);
                //cout << "imshow procS " << endl;

                //cout << MatObjectDetected << endl;
                if(!MatObjectDetected.empty())
                {
#ifdef controlb
                if(isControlB){
                if(withMovement == 0)
                {
                 controlB->process(0,false,MatObjectDetected,boolbrake);
                }else if (withMovement == 1)
                {
                 controlB->process(0,true,MatObjectDetected,boolbrake);
                }
                if(movementProcessS){
                controlB->process(0,true,MatObjectDetected,boolbrake);

                }
                }
#endif
                }
                }
                }

#endif

#ifdef controlUInt // toto je koncept navigacie s vyhybanim sa prekazkam a navigaciou pomocou ciar (neprejavil sa ako korektne fungujuci)
                if(!frameDisparity.empty()){
                processS->process(frameDisparity, frameRight);
                frameprocessSBlizke = processS->getFrame();
                frameprocessSStredne = processS->getDisparity();

                processC->process(frameIPM, frameRight);
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
                          if(!frameprocessSBlizke.empty() && !frameprocessSStredne.empty())
                          {
                               cv::imshow("DisparitaBlizke",frameprocessSBlizke);
                               cv::imshow("DisparitaStredne",frameprocessSStredne);
                               MatObjectDetected = processS->getObject();
                            if(!MatObjectDetected.empty())
                            {
                            if(withMovement == 0)
                            {
                               controlUI->process(iUholProcessC,false,MatObjectDetected,boolbrake);
                            }else if (withMovement == 1)
                            {
                               controlUI->process(iUholProcessC,true,MatObjectDetected,boolbrake);
                            }
                            if(movementProcessS){
                               controlUI->process(iUholProcessC,true,MatObjectDetected,boolbrake);
                            }
                            }
                          }
                          }

                          //lockThread.try_lock();
					frameProcessC = processC->getFrame();
					//lockThread.unlock();
					if (!frameProcessC.empty())
					{
						imshow("ProcessC", frameProcessC);
					}
					else { cout << "Frame processC is Empty!" << endl; }

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
					if(isFrameCapture)
					{
                                            if(bitSaveFrameDisparity == 1)
                                              {
                    milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
                    std::string str = to_string(ms.count());
                    std::string meno = str + ".jpeg";
                    cv::imwrite(strPathFrameDisparity+ meno,frameDisparity);}
					}

#ifdef procBGS
					if(!backgroundImage.empty()){
					processBGS->process(frameDisparity, backgroundImage);
					frameBGS = processBGS->getFrame();
					MatObjectDetected = processBGS->getObject();

					imshow("FrameBGS", frameBGS);
					if(!MatObjectDetected.empty())
					{
					cout << MatObjectDetected << endl;
					}

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
					//imshow("MediumDistanceObj", frameMediumObj);
				}

				if (!frameMediumObj.empty())
				{
					//imshow("FarDistanceObj", frameFarObj);
				}

				if (!frameIPM.empty())
				{
					imshow("InversePerspectiveMapping", frameIPM);

#ifdef procc

                    processC->process(frameIPM, frameRight);
                    int iUholProcessC;
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
                          iUholProcessC = (int)round(uholProcessC);
                          if(iUholProcessC != iTmpUholProcessC)
                          {
                            controlA->process(iUholProcessC,withMovement,MatObjectDetected,boolbrake);
                            //cout << "Uhol:" << iUholProcessC << endl;
                          }
                          iTmpUholProcessC = iUholProcessC; // pamatanie


                          }


                    }
					//lockThread.try_lock();
					frameProcessC = processC->getFrame();
                    frameProcessCThresh = processC->getDisparity();
					//lockThread.unlock();
					if (!frameProcessC.empty())
					{
						imshow("ProcessC", frameProcessC);
						if(isFrameCapture)
					{
                                if(bitSaveFrameProcC == 1){
                                              string napisFPSprocC = to_string(iUholProcessC);
					      cv::putText(frameProcessC,napisFPSprocC,cvPoint(0,25),cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(255,255,255), 2);
                                              milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
                                              std::string str = to_string(ms.count());
                                              std::string meno = str + ".jpeg";
                                              cv::imwrite(strPathFrameProcCLane+ meno,frameProcessC);
                                              cv::imwrite(strPathFrameProcCThresh+ meno,frameProcessCThresh);}

                                if(bitSaveFrameIPM == 1){


                                              milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
                                              std::string str = to_string(ms.count());
                                              std::string meno = str + ".jpeg";
                                              cv::imwrite(strPathFrameIPM + meno,frameIPM);
                                              }
					}
					}
					else { cout << "Frame processC is Empty!" << endl; }


#endif


				}
			}
			else { cout << "Frame Left or Right are Empty!" << endl; }
#ifdef prock
					processK->process(frameLeft, frameLeft);
					frameCascade = processK->getFrame();
					if (!frameCascade.empty())
					{
						imshow("Lane_Cascade", frameCascade);
					}
					else { cout << "Frame Lane_Cascade is Empty!" << endl; }
#endif
			//cv::waitKey(1);
			//printf("asdf  ");
			//cout << "Check klavesnica" << endl;

			char key = (char)cv::waitKey(5);
			//cout << "waitkey" << endl;
			switch (key)
			{

            case 'o': {printf("skusobny vypis");

                                  break;}
            case 'c': {if(isVideoCapture)//nahravanie pomocou videorecordera, ktore neslo...
                        {
                        isVideoCapture = false;

                        outputFrameLeft.release();
                        cout << "Nahravanie OFF" << endl;
                        }
                        else{
                        isVideoCapture = true;
                        cout << "Nahravanie ON" << endl;
                        }
                                  break;}
            case 's': {if(boolbrake)// posle signal brzdy do auticka
                        {
                        boolbrake = false;
                        cout << "BRAKE DEACTIVATED" << endl;
                        }
                        else{
                        boolbrake = true;
                        cout << "BRAKE ACTIVATED" << endl;
                        }
                                  break;}
            case 'x': {if(movementProcessS)
                        {
                        movementProcessS = false;
                        cout << "ProcessS ovladanie OFF" << endl;
                        }
                        else{
                        movementProcessS = true;
                        cout << "ProcessS ovladanie ON" << endl;
                        }
                                  break;}

            case 'l': {if(isControlB) // zapnutie controluB pre ovladanie natocenia kolies od procS (vyhybanie sa prekazkam)
                        {
                        isControlB = false;
                        cout << "ControlB ovladanie OFF" << endl;
                        }
                        else{
                        isControlB = true;
                        cout << "ControlB ovladanie ON" << endl;
                        }
                                  break;}
            case 'k': {if(servoControl) // zapnutie navigacie pomocou ciar
                        {
                        servoControl = false;
                        cout << "Servo ovladanie OFF" << endl;
                        }
                        else{
                        servoControl = true;
                        cout << "Servo ovladanie ON" << endl;
                        }
                                  break;}
            case 'r': {if(isFrameCapture) // nahravanie samostatnych frameov do zlozky saved_frames (nastavenia v configu)
                        {
                        isFrameCapture = false;
                        cout << "Nahravanie obrazkov OFF" << endl;
                        }
                        else{
                        isFrameCapture = true;
                        cout << "Nahravanie obrazkov ON" << endl;
                        }
                                  break;}

            case 'm': {if(withMovement == 1)// spustenie auticka smer dopredu s motorom
                        {
                        withMovement = 0;
                        cout << "Motor vpred OFF" << endl;
                        }
                        else{
                        withMovement = 1;
                        cout << "Motor vpred ON" << endl;
                        }
                                  break;}
            case 'w':    // zapnutie s motorom smerom dopredu navigaciou podla cestnych ciar
                          {if(withMovement == 0 && servoControl== false)
                                  {controlA->process(uholNatocenia,false,MatObjectDetected,boolbrake);}
                                  else if(withMovement == 1 && servoControl== false)
                                  {controlA->process(uholNatocenia,true,MatObjectDetected,boolbrake);}
                                  break;}
            case 'a': {if(uholNatocenia >= 3)// klavesove natocenia kolies o 3 stupne (a-vlavo)
                                  {uholNatocenia = uholNatocenia -3;
                                  }
                                  controlA->process(uholNatocenia,false,MatObjectDetected,boolbrake);
                                  break;}
            case 'd': {if(uholNatocenia <= 57)// klavesove natocenia kolies o 3 stupne (D-vpravo)
                                  {uholNatocenia= uholNatocenia +3;
                                  }
                                  controlA->process(uholNatocenia,false,MatObjectDetected,boolbrake);
                                  break;}
	    case 'i': // ulozenie aktualnej disparity
                      {imwrite("disparita.png",frameDisparity);
                                  break;}

            case 'b':// ulozenie aktualnej disparity
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
		outputFrameLeft.release();
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
