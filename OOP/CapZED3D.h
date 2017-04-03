
#include "ICapture.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>
using namespace boost;
class CapZED3D: public ICapture {
	

public:
	
	Mat   frame[2], display[2], frameGray[2], frameLeftFilterB, frameRightFilterB;
	Mat rgb, depth, object;
	VideoCapture capture[2];
	Size size;
	int sigmaColor, sigmaSpace, borderType, erodeSizeW, erodeSizeH, dilateSizeW, dilateSizeH, grayScale;
	property_tree::ptree pt; // citac .ini suborov
	//haluz

	// parameters are videoFile(path to file) 
	CapZED3D(string pPath1, string pPath2)
	{
		capture[0].open(pPath1);
		capture[1].open(pPath2);
		size = Size(480,320);
		property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			erodeSizeW = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.erodeSizeW"));
			erodeSizeH = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.erodeSizeH"));
			dilateSizeW = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.dilateSizeW"));
			dilateSizeH = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.dilateSizeH"));
			sigmaColor = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.sigmaColor"));
			sigmaSpace = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.sigmaSpace"));
			borderType = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.borderType"));
			grayScale = boost::lexical_cast<int>(pt.get<string>("VideoInput.GrayScale"));
		}
		catch (...)
		{
			cout << "Error in parsing Filter in CaptureZen3D!" << endl;
		}
		cvNamedWindow("InputFilter control", CV_WINDOW_AUTOSIZE);
		resizeWindow("InputFilter control", 400, 500);
		moveWindow("InputFilter control", 800, 0);
		//createButton("InputFilter controlaaa", callbackButton, NULL, QT_CHECKBOX, 0);
		
	}
	// parameters are usb input(number)
	CapZED3D(int pPath1, int pPath2)
	{
		capture[0].open(pPath1);
		capture[1].open(pPath2);
		//capture[0].set(CV_CAP_PROP_FPS, 5); // aj tak zrejme nejde
		//capture[1].set(CV_CAP_PROP_FPS, 5);
		
		size = Size(480, 320);
		cvNamedWindow("InputFilter control", CV_WINDOW_AUTOSIZE);
		resizeWindow("InputFilter control", 400, 500);
		moveWindow("InputFilter control", 800, 0);
	}

	~CapZED3D()
	{}

	virtual void process()
	{
		cvCreateTrackbar("GrayScale", "InputFilter control", &grayScale, 1);
		cvCreateTrackbar("W-Erode", "InputFilter control", &erodeSizeW, 15);
		cvCreateTrackbar("H-Erode", "InputFilter control", &erodeSizeH, 15);
		cvCreateTrackbar("W-Dilate", "InputFilter control", &dilateSizeW, 15);
		cvCreateTrackbar("H-Dilate", "InputFilter control", &dilateSizeH, 15);
		cvCreateTrackbar("SigmaColor", "InputFilter control", &sigmaColor, 15);
		cvCreateTrackbar("SigmaSpace", "InputFilter control", &sigmaSpace, 100);
		cvCreateTrackbar("BorderType", "InputFilter control", &borderType, 100);

		if (sigmaColor < 1)sigmaColor = 1;
		omp_set_num_threads(2);
#pragma omp parallel
			{
				int TID = omp_get_thread_num();
				capture[TID].read(frame[TID]);
				if (frame[TID].empty()) {
					cout << "!!!!!!!!Empty frame!!!!!!!!" + TID << endl;
				}
				else{
					grayScale = 0;      // odstranit ak sa nejak opravi grayScale, lebo zatial nejde
					if (grayScale == 1)
					{	
						cvtColor(frame[TID], frame[TID], CV_BGR2GRAY);
						erode(frame[TID], frame[TID], getStructuringElement(MORPH_RECT, Size(erodeSizeW, erodeSizeH)));
						dilate(frame[TID], frame[TID], getStructuringElement(MORPH_RECT, Size(dilateSizeW, dilateSizeH)));
						resize(frame[TID], frame[TID], size);
						//frameGray[TID].convertTo(frame[TID], CV_8UC3, 1);
						bilateralFilter(frame[TID], display[TID], sigmaColor, sigmaSpace, borderType);
					}
					else{
						erode(frame[TID], frame[TID], getStructuringElement(MORPH_RECT, Size(erodeSizeW, erodeSizeH)));
						dilate(frame[TID], frame[TID], getStructuringElement(MORPH_RECT, Size(dilateSizeW, dilateSizeH)));
						resize(frame[TID], frame[TID], size);
						bilateralFilter(frame[TID], display[TID], sigmaColor, sigmaSpace, borderType);
					}
				}
#pragma omp barrier
			}
			
			
	}

	virtual Mat getLeftRGB()
	{
		return display[0];
	}
	virtual Mat getRightRGB()
	{
		return display[1];
	}

	virtual Mat getDepthMap()
	{
		Mat pom;
		return pom;
	}
	/*static void callbackButton(int state, void *userdata)
	{

	}*/
	

};