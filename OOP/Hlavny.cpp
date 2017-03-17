
#include <stdio.h>
#include <string>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>



#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"

#include "Structures.h"
#include "DepthMap.h"
#include "Calibration.h"
#include "LineAssist.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

int enGeomCal = 1, enGrayConv = 1, enFilter = 1, enDisparity = 1, enDetectObject = 1, enLaneAssist = 1, first = 1;


//variables
int menu = 0; //choose trackbars
int s = 0;
int height;//parser.get<int>(2);
int &r_height = height;
int width;//parser.get<int>(3);
int &r_width = width;
int fps;//parser.get<int>(4);
int &r_fps = fps;

string IntToStr(int n);
void setStartingParameters(int argc, char *argv[]);
void setCaptures(VideoCapture *p_CapL, VideoCapture *p_CapR, int &r_height, int &r_width, int &r_fps);
void filterFrames(DepthMap *p_DFrameL, DepthMap *p_DFrameR);
void setFrames(DepthMap *p_DFrameL, DepthMap *p_DFrameR, int &r_width, int &r_height);
void createDisparityMap(DepthMap *p_disparityFrame, DepthMap *p_DFrameL, DepthMap *p_DFrameR);
void windowsPosition();
void windowSettings();
void showMenu();
void screenFrames(LineAssist *p_LFrameL, LineAssist *p_LFrameR, string fileNameL, string fileNameR);


//open camera bus connection
VideoCapture captureL;
VideoCapture *pntrCapL;
VideoCapture captureR;
VideoCapture *pntrCapR;

rotatKoef rotation;
rotatKoef *pntrRotation;
disparityKoef disparity;
disparityKoef *pntrDisparity;
filterKoef filter;
filterKoef *pntrFilter;
laneAssistKoef laneAssist;
laneAssistKoef *pntrLaneAssist;

/**
*argumenty main funkcie
**/
const char* keys =
{
	"{@camera_number01| C:\\Users\\Gamer\\Desktop\\video\\left.mp4   |camera number}"
	"{@camera_number02| C:\\Users\\Gamer\\Desktop\\video\\right.mp4   |camera number}"
	"{@height         | 320 |camera height}"
	"{@width          | 240 |camera width }"
	"{@fps            | 30  |camera fps   }"
};/*inicializacne hodnoty*/




int main(int argc, char *argv[])
{

	//parsing main function
	auto then = system_clock::now();
	setStartingParameters(argc, argv);
	auto now = system_clock::now();
	cout << "Execution Time: " << duration_cast<milliseconds>(now - then).count() << " MiliSeconds" << endl;


	//set camera parameters,
	setCaptures(pntrCapL, pntrCapR, r_height, r_width, r_fps);

	//create strcture objects

	pntrRotation = &rotation;
	pntrDisparity = &disparity;
	pntrFilter = &filter;
	pntrLaneAssist = &laneAssist;

	//create trackbars gui for structure objects
	pntrRotation->readRotatKoef();
	pntrDisparity->readDisparityKoef();
	pntrFilter->readFilterKoef();
	//set after start

	//never ending story
	while (1)
	{
		auto start = system_clock::now();
		//get frames
		DepthMap dFrameL = DepthMap(captureL);
		DepthMap *pntrDFrameL = &dFrameL;

		DepthMap dFrameR = DepthMap(captureR);
		DepthMap *pntrDFrameR = &dFrameR;

		LineAssist lFrameL = LineAssist(captureL);
		LineAssist *pntrLFrameL = &lFrameL;

		LineAssist lFrameR = LineAssist(captureR);
		LineAssist *pntrLFrameR = &lFrameR;

		Calibration cFrameL = Calibration(captureL);
		Calibration *pntrCFrameL = &cFrameL;

		Calibration cFrameR = Calibration(captureR);
		Calibration *pntrCFrameR = &cFrameR;


		if (pntrDFrameL->getFrame().empty() != true && pntrDFrameR->getFrame().empty() != true)
		{

			//geometric calibration
			if (r_height == 320 && r_width == 240 && enGeomCal == 1)
			{
				


				auto then = system_clock::now();

				//thread third(pntrRotation, pntrRotation->M1, pntrRotation->D1, pntrRotation->R1, pntrRotation->P1);

				*pntrCFrameL = pntrCFrameL->calibFrame(*pntrCFrameL, pntrRotation->M1, pntrRotation->D1, pntrRotation->R1, pntrRotation->P1);
				*pntrCFrameR = pntrCFrameR->calibFrame(*pntrCFrameR, pntrRotation->M2, pntrRotation->D2, pntrRotation->R2, pntrRotation->P2);
				auto now = system_clock::now();
				cout << "Execution Time GEOMETRIC CALIBRATION: " << duration_cast<milliseconds>(now - then).count() << " MiliSeconds" << endl;

			}
			//convert rgb to gray for better computing
			if (enGrayConv == 1)
			{
				auto then = system_clock::now();
				setFrames(pntrDFrameL, pntrDFrameR, r_width, r_height);
				auto now = system_clock::now();
				cout << "Execution Time RGB TO GRAY: " << duration_cast<milliseconds>(now - then).count() << " MiliSeconds" << endl;
			}
			//filtering input images
			if (enFilter == 1)
			{
				auto then = system_clock::now();
				*pntrDFrameL = pntrDFrameL->filterFrame(*pntrDFrameL, *pntrFilter);
				*pntrDFrameR = pntrDFrameR->filterFrame(*pntrDFrameR, *pntrFilter);
				imshow("frameL", pntrDFrameL->getFrame());
				imshow("frameR", pntrDFrameR->getFrame());
				auto now = system_clock::now();
				cout << "Execution Time FILTER: " << duration_cast<microseconds>(now - then).count() << " MicroSeconds" << endl;
			}
			else
			{
				destroyWindow("frameL");
				destroyWindow("frameR");
			}
			// creating disparity map from input frames
			if (enDisparity == 1)
			{
				auto then = system_clock::now();
				DepthMap disparityFrame;
				DepthMap *pointerDisparityFrame = &disparityFrame;
				createDisparityMap(pointerDisparityFrame, pntrDFrameL, pntrDFrameR);
				imshow("disparity", pointerDisparityFrame->getFrame());
				if (enDetectObject == 1)
				{
					DepthMap reductedDisparity;
					DepthMap *pointerReductedDisparity = &reductedDisparity;
					Mat reductedImage;
					Mat *pointerReductedImage = &reductedImage;
					*pointerReductedImage = pointerDisparityFrame->getFrame();
					inRange(*pointerReductedImage, pntrDisparity->rangeDispLow, pntrDisparity->rangeDispHigh, *pointerReductedImage);
					pointerReductedDisparity->setFrame(*pointerReductedImage);
					imshow("ranged Disparity", pointerReductedDisparity->getFrame());
				}
				else
				{
					destroyWindow("ranged Disparity");
				}
				auto now = system_clock::now();
				cout << "Execution Time DISPARITY: " << duration_cast<milliseconds>(now - then).count() << " MiliSeconds" << endl;
			}
			else
			{
				enDetectObject = 0;
				destroyWindow("ranged Disparity");
				destroyWindow("disparity");
			}
		}
		else
		{
			destroyAllWindows();
			return 0;
		}

		if (pntrDFrameL->getFrame().empty() != true && pntrDFrameR->getFrame().empty() != true)
		{
			if (enLaneAssist == 1)
			{
				auto then = system_clock::now();
				Calibration laneAssistFrame1;
				Calibration *pointerLaneAssistFrame1 = &laneAssistFrame1;
				Calibration laneAssistFrame;
				Calibration *pointerLaneAssistFrame = &laneAssistFrame;
				*pointerLaneAssistFrame = pntrCFrameR->birdViewTransform(*pntrCFrameR, *pntrLaneAssist);
				imshow("BirdView", pointerLaneAssistFrame->getFrame());
				//laneAssistFrame1=laneAssistFrame1.laneDetect(laneAssistFrame,laneAssist);
				// imshow("Lane Detect",laneAssistFrame1.getFrame());
				auto now = system_clock::now();
				cout << "Execution Time LINE ASSIST: " << duration_cast<milliseconds>(now - then).count() << " MiliSeconds" << endl;

			}
			else
			{
				destroyWindow("BirdView");
				destroyWindow("Lane Detect");
			}
		}
		else
		{
			destroyAllWindows();
			return 0;
		}

		auto then = system_clock::now();
		if (first == 1)
		{
			windowsPosition();
			first = 0;
		}


		switch (menu)
		{
		case 0:
			pntrFilter->setFilterKoef();
			break;
		case 1:
			destroyWindow("Filters");
			pntrDisparity->setDisparityKoef();
			break;
		case 2:
			destroyWindow("StereoSGBM control");
			pntrLaneAssist->setLaneAssistKoef();
			break;
		case 3:
			destroyWindow("LaneAssist control");
			windowSettings();
			break;
		case 4:
			destroyWindow("Filters");
			destroyWindow("StereoSGBM control");
			destroyWindow("LaneAssist control");
			destroyWindow("Function control");
			break;
		}

		moveWindow("Filters", 975, 0);
		moveWindow("StereoSGBM control", 975, 0);
		moveWindow("LaneAssist control", 975, 0);
		moveWindow("Function control", 975, 0);

		auto now = system_clock::now();
		cout << "Execution Time MOVE WINDOWS: " << duration_cast<milliseconds>(now - then).count() << " MiliSeconds" << endl;

		string filename0, filename1;

		char key = (char)waitKey(30);
		char*pointerKey = &key;
		switch (*pointerKey)
		{
		case 's':
		case 'S':
		{

			screenFrames(pntrLFrameL, pntrLFrameR, filename0, filename1);
			s++;
		}
			break;
		case 'q':
		case 'Q':
		case  27: //escape key
		{
			return 0;
		}
			break;
		case 't':
		case 'T':
		{
			menu++;
			if (menu > 4) menu = 0;
		}
			break;
		case 'w':
		case 'W':
		{
			pntrDisparity->writeDisparityKoef();
			pntrFilter->writeFilterKoef();
			pntrLaneAssist->writeLaneAssistKoef();
		}
			break;
		case 'r':
		case 'R':
		{
			pntrDisparity->readDisparityKoef();
			pntrFilter->readFilterKoef();
			pntrLaneAssist->readLaneAssistKoef();
		}
			break;
		default:
			break;
		}
		auto end = system_clock::now();
		cout << "Elapsed Time PER CYCLE: " << duration_cast<milliseconds>(end - start).count() << " MiliSeconds" << endl;
	}
	return 0;
}

void setStartingParameters(int argc, char *argv[])
{
	CommandLineParser parser(argc, argv, keys);
	string sourceL = parser.get<string>(0);
	string sourceR = parser.get<string>(1);
	height = parser.get<int>(2);
	width = parser.get<int>(3);
	fps = parser.get<int>(4);

	//open camera bus connection
	captureL = VideoCapture(sourceL);
	captureR = VideoCapture(sourceR);
	pntrCapL = &captureL;
	pntrCapR = &captureR;
	//if not open, convert string to int and try again
	if (!pntrCapL->isOpened())
	{
		//pntrCapL->open(atoi(sourceL.c_str()));
	}

	if (!pntrCapR->isOpened())
	{
		//pntrCapR->open(atoi(sourceR.c_str()));
	}
}

void setCaptures(VideoCapture *p_CapL, VideoCapture *p_CapR, int &r_height, int &r_width, int &r_fps)
{
	p_CapL->set(CV_CAP_PROP_FRAME_HEIGHT, r_height);
	p_CapL->set(CV_CAP_PROP_FRAME_WIDTH, r_width);
	p_CapL->set(CV_CAP_PROP_FPS, r_fps);
	p_CapR->set(CV_CAP_PROP_FRAME_HEIGHT, r_height);
	p_CapR->set(CV_CAP_PROP_FRAME_WIDTH, r_width);
	p_CapR->set(CV_CAP_PROP_FPS, r_fps);
}


void setFrames(DepthMap *p_DFrameL, DepthMap *p_DFrameR, int &r_width, int &r_height)
{
	Mat grayL = p_DFrameL->getFrame();
	Mat grayR = p_DFrameR->getFrame();
	cvtColor(grayL, grayL, COLOR_BGR2GRAY);
	cvtColor(grayR, grayR, COLOR_BGR2GRAY);
	resize(grayL, grayL, Size(320, 240), 0, 0);
	resize(grayR, grayR, Size(320, 240), 0, 0);
	p_DFrameL->setFrame(grayL);
	p_DFrameR->setFrame(grayR);
}

void createDisparityMap(DepthMap *p_disparityFrame, DepthMap *p_DFrameL, DepthMap *p_DFrameR)
{
	*p_disparityFrame = p_disparityFrame->stereoCalc(*p_DFrameL, *p_DFrameR, *pntrDisparity);
	*p_disparityFrame = p_disparityFrame->resFilterFrame(*p_disparityFrame, *pntrFilter);
	Mat croped;
	Mat *pointerCroped = &croped;
	*pointerCroped = p_disparityFrame->getFrame()(Rect(30, 40, 270, 190)); //crop disparity, just calibrated ROI
	p_disparityFrame->setFrame(*pointerCroped);
}

void windowSettings()
{
	cvNamedWindow("Function control", CV_WINDOW_NORMAL);
	cvCreateTrackbar("enGeomCal", "Function control", &enGeomCal, 1);
	cvCreateTrackbar("enGrayConv", "Function control", &enGrayConv, 1);
	cvCreateTrackbar("enFilter", "Function control", &enFilter, 1);
	cvCreateTrackbar("enDisparity", "Function control", &enDisparity, 1);
	cvCreateTrackbar("enDetectObject", "Function control", &enDetectObject, 1);
	cvCreateTrackbar("enLaneAssist", "Function control", &enLaneAssist, 1);
	cvCreateTrackbar("Restart position", "Function control", &first, 1);
}

void windowsPosition(){
	if (first = 1)
	{
		moveWindow("frameL", 0, 0);
		moveWindow("frameR", 325, 0);
		moveWindow("disparity", 650, 0);
		moveWindow("ranged Disparity", 650, 270);
		moveWindow("Lane Detect", 0, 270);
		moveWindow("BirdView", 325, 270);
	}

	moveWindow("Filters", 975, 0);
	moveWindow("StereoSGBM control", 975, 0);
	moveWindow("LaneAssist control", 975, 0);
	moveWindow("Function control", 975, 0);
}

void showMenu(){

	switch (menu)
	{
	case 0:
		pntrFilter->setFilterKoef();
		break;
	case 1:
		destroyWindow("Filters");
		pntrDisparity->setDisparityKoef();
		break;
	case 2:
		destroyWindow("StereoSGBM control");
		pntrLaneAssist->setLaneAssistKoef();
		break;
	case 3:
		destroyWindow("LaneAssist control");
		windowSettings();
		break;
	case 4:
		destroyWindow("Filters");
		destroyWindow("StereoSGBM control");
		destroyWindow("LaneAssist control");
		destroyWindow("Function control");
		break;
	}

}

void screenFrames(LineAssist *p_LFrameL, LineAssist *p_LFrameR, string fileNameL, string fileNameR)
{
	fileNameL = "data/pos" + IntToStr(s) + ".png";
	fileNameR = "data/crop" + IntToStr(s) + ".png";
	imwrite(fileNameL, p_LFrameL->getFrame());
	imwrite(fileNameR, p_LFrameR->getFrame());
}

string IntToStr(int n)
{// prevod cisla na retazec znakov, ukladanie obrazkov
	stringstream result;
	result << n;
	return result.str();
}

