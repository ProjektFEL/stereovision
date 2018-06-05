
#include "ICapture.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <zed/Mat.hpp>
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>
#include "kernel.cuh"
#ifndef _SL_JETSON_   // defined in zed/utils/GlobalDefines.hpp --> Detect if we are running under a Jetson TK1 or TX1
#include <opencv2/core/utility.hpp>
#endif

using namespace boost;
using namespace std;

using namespace sl::zed;
using namespace sl;
#include <iostream>
class CapZED3D: public ICapture {


public:

	cv::Mat   frame[2], display[2], frameGray[2], frameLeftFilterB, frameRightFilterB;
	cv::Mat rgb, depth, object;
	cv::VideoCapture capture[2];
	cv::Size size;
	int sigmaColor, sigmaSpace, borderType, erodeSizeW, erodeSizeH, dilateSizeW, dilateSizeH, grayScale, isCameraZed, isOpenmpEnabled;
	property_tree::ptree pt; // citac .ini suborov
	bool isInputCamera;
	cv::Mat M[2], D[2], R[2], P[2];
	cv::Mat imageZedLeft;

	cv::Mat imageZedRight;
	cv::Mat imageZedDepth;
	GrabParams runtime_parameters;
	// Create a ZED camera object
	Camera* zed;
sl::zed::Mat imageLeft, imageRight,  point_cloud, imageDepth;
	int debugMode, confidenceThreshold;
	bool onlyFirstTime = false;

	// parameters are videoFile(path to file)
	CapZED3D(string pPath1, string pPath2)
	{
		isInputCamera = false;
		cout << pPath1 << endl;
		capture[0].open(pPath1);
		capture[1].open(pPath2);
		size = cv::Size(480,320);
		property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			isOpenmpEnabled = boost::lexical_cast<int>(pt.get<string>("VideoInput.isOpenmp"));
			erodeSizeW = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.erodeSizeW"));
			erodeSizeH = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.erodeSizeH"));
			dilateSizeW = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.dilateSizeW"));
			dilateSizeH = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.dilateSizeH"));
			sigmaColor = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.sigmaColor"));
			sigmaSpace = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.sigmaSpace"));
			borderType = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.borderType"));
			grayScale = boost::lexical_cast<int>(pt.get<string>("VideoInput.GrayScale"));
                        confidenceThreshold = boost::lexical_cast<int>(pt.get<string>("VideoInput.ConfidenceThreshold"));
		}
		catch (...)
		{
			cout << "Error in parsing Filter in CaptureZen3D!" << endl;
		}
		cvNamedWindow("InputFilter control", CV_WINDOW_AUTOSIZE);
		cv::resizeWindow("InputFilter control", 400, 500);
		cv::moveWindow("InputFilter control", 800, 0);
		//createButton("InputFilter controlaaa", callbackButton, NULL, QT_CHECKBOX, 0);
		cout << "objekt kamera zo suboru" <<endl;

	}
	// parameters are usb input(number)
	CapZED3D(int pPath1, int pPath2)
	{
                
		property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			isCameraZed = boost::lexical_cast<int>(pt.get<string>("VideoInput.CameraZed"));
			isOpenmpEnabled = boost::lexical_cast<int>(pt.get<string>("VideoInput.isOpenmp"));
			erodeSizeW = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.erodeSizeW"));
			erodeSizeH = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.erodeSizeH"));
			dilateSizeW = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.dilateSizeW"));
			dilateSizeH = boost::lexical_cast<int>(pt.get<string>("Erode&DilateInput.dilateSizeH"));
			sigmaColor = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.sigmaColor"));
			sigmaSpace = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.sigmaSpace"));
			borderType = boost::lexical_cast<int>(pt.get<string>("BilateralFilter.borderType"));
			grayScale = boost::lexical_cast<int>(pt.get<string>("VideoInput.GrayScale"));
			debugMode = boost::lexical_cast<int>(pt.get<string>("VideoInput.debugMode"));
                        confidenceThreshold = boost::lexical_cast<int>(pt.get<string>("VideoInput.ConfidenceThreshold"));
		}
		catch (...)
		{
			cout << "Error in parsing Filter in CaptureZen3D!" << endl;
		}

		if (isCameraZed == 0){ // obycajna kamera s kalibraciou
		isInputCamera = true;
		capture[0].open(pPath1);
		capture[1].open(pPath2);
		//capture[0].set(CV_CAP_PROP_FPS, 5); // aj tak zrejme nejde
		//capture[1].set(CV_CAP_PROP_FPS, 5);

		size = cv::Size(480, 320);

		
		//cv::resizeWindow("InputFilter control", 400, 500);
		//cv::moveWindow("InputFilter control", 800, 0);
		cv::FileStorage fs1("intrinsics.yml", CV_STORAGE_READ);
		cv::Mat pom;
		fs1["M1"] >> M[0];
		fs1["M2"] >> M[1];
		fs1["D1"] >> D[0];
		fs1["D2"] >> D[1];
		fs1.release();

		cv::FileStorage fs2("extrinsics.yml", CV_STORAGE_READ);
		fs2["R1"] >> R[0];
		fs2["R2"] >> R[1];
		fs2["P1"] >> P[0];
		fs2["P2"] >> P[1];

		fs2.release();
		}
		else { // camera zed
			// Create a ZED camera object

			zed = new Camera(VGA);

			// Set configuration parameters
			InitParams init_params;

			//init_params.camera_resolution = ZEDResolution_mode::VGA;

			init_params.mode = sl::zed::MODE::PERFORMANCE;
                        init_params.minimumDistance = 600;
            init_params.unit = sl::zed::UNIT::MILLIMETER;
            //init_params.device = -1;

			init_params.verbose = 1;
			//init_params.svo_real_time_mode = true;

															// Open the camera
			ERRCODE err = zed->init(init_params);
    if (err != SUCCESS)
        exit(-1);

			// Set runtime parameters after opening the camera

			//runtime_parameters.dm_type = STANDARD; // Use STANDARD sensing mode


		}
	}

	~CapZED3D()
	{}

	virtual void process()
	{
                cv::namedWindow("InputFilter control", CV_WINDOW_AUTOSIZE);		
                cvCreateTrackbar("GrayScale", "InputFilter control", &grayScale, 1);
		cvCreateTrackbar("W-Erode", "InputFilter control", &erodeSizeW, 15);
		cvCreateTrackbar("H-Erode", "InputFilter control", &erodeSizeH, 15);
		cvCreateTrackbar("W-Dilate", "InputFilter control", &dilateSizeW, 15);
		cvCreateTrackbar("H-Dilate", "InputFilter control", &dilateSizeH, 15);
		cvCreateTrackbar("SigmaColor", "InputFilter control", &sigmaColor, 15);
		cvCreateTrackbar("SigmaSpace", "InputFilter control", &sigmaSpace, 100);
		cvCreateTrackbar("BorderType", "InputFilter control", &borderType, 100);
                cvCreateTrackbar("ConfidenceThreshold", "InputFilter control", &confidenceThreshold, 100);
		if (sigmaColor < 1)sigmaColor = 1;

		if (isOpenmpEnabled == 0 && isCameraZed == 0)
		{
			cout << "kamera subor" << endl;
			capture[0].read(frame[0]);
			capture[1].read(frame[1]);

			erode(frame[0], frame[0], getStructuringElement(cv::MORPH_RECT, cv::Size(erodeSizeW, erodeSizeH)));
			dilate(frame[0], frame[0], getStructuringElement(cv::MORPH_RECT, cv::Size(dilateSizeW, dilateSizeH)));
			erode(frame[1], frame[1], getStructuringElement(cv::MORPH_RECT, cv::Size(erodeSizeW, erodeSizeH)));
			dilate(frame[1], frame[1], getStructuringElement(cv::MORPH_RECT, cv::Size(dilateSizeW, dilateSizeH)));
			//resize(frame[0], frame[0], size);
			//resize(frame[1], frame[1], size);
			//frameGray[TID].convertTo(frame[TID], CV_8UC3, 1);
			bilateralFilter(frame[0], display[0], sigmaColor, sigmaSpace, borderType);
			bilateralFilter(frame[1], display[1], sigmaColor, sigmaSpace, borderType);
		}

		if (isCameraZed == 0 || isOpenmpEnabled == 1) {

/*

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
						if (isInputCamera){
							frame[TID] = calibFrame(frame[TID], M[TID], D[TID], R[TID], P[TID]);
						}
						cvtColor(frame[TID], frame[TID], CV_BGR2GRAY);
						erode(frame[TID], frame[TID], getStructuringElement(cv::MORPH_RECT, cv::Size(erodeSizeW, erodeSizeH)));
						dilate(frame[TID], frame[TID], getStructuringElement(cv::MORPH_RECT, cv::Size(dilateSizeW, dilateSizeH)));
						resize(frame[TID], frame[TID], size);
						//frameGray[TID].convertTo(frame[TID], CV_8UC3, 1);
						bilateralFilter(frame[TID], display[TID], sigmaColor, sigmaSpace, borderType);
					}
					else{

						erode(frame[TID], frame[TID], getStructuringElement(cv::MORPH_RECT, cv::Size(erodeSizeW, erodeSizeH)));
						dilate(frame[TID], frame[TID], getStructuringElement(cv::MORPH_RECT, cv::Size(dilateSizeW, dilateSizeH)));
						resize(frame[TID], frame[TID], size);
						if (isInputCamera){
							//calibFrame(frame[TID], M[TID], D[TID], R[TID], P[TID]).copyTo(frame[TID]);
							frame[TID] = calibFrame(frame[TID], M[TID], D[TID], R[TID], P[TID]);
						}
						bilateralFilter(frame[TID], display[TID], sigmaColor, sigmaSpace, borderType);
					}
				}
#pragma omp barrier
			}*/

		}
		else if (isCameraZed == 1){
zed->setConfidenceThreshold(confidenceThreshold);

			if (zed->grab(runtime_parameters) == SUCCESS) {

//zed->setCameraSettingsValue(ZED_SATURATION,1,false);
int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;
    cv::Size size(width, height); // image size

    //imageZedDepth = new Mat(size, CV_8UC1); // normalized depth to display
    int depth_clamp = 4000;
    zed->setDepthClampValue(depth_clamp);
    //zed->sticktoCPUCore(3);

				// Grab the current images and compute the disparity
        bool res = zed->grab(STANDARD, 1, 1, 0);
imageLeft =zed->retrieveImage(sl::zed::SIDE::LEFT);
sl::zed::slMat2cvMat(imageLeft).copyTo(display[0]);
imageRight =zed->retrieveImage(sl::zed::SIDE::RIGHT);
	sl::zed::slMat2cvMat(imageRight).copyTo(display[1]);
slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(imageZedDepth);
  	//imageDepth = zed->retrieveMeasure(DISPARITY);


			}

//sl::zed::slMat2cvMat(imageDepth).copyTo(imageZedDepth);
			/*  debug mode a vypisky, treba doriesit potom
			if(debugMode == 1 && onlyFirstTime == false){
				string tyLeft = type2str(display[0].type());
				string tyRight = type2str(display[1].type());
				string tyDepth = type2str(imageZedDepth.type());
				std::printf("Left: %s %dx%d \n", tyLeft.c_str(), display[0].cols, display[0].rows);
				std::printf("Right: %s %dx%d \n", tyRight.c_str(), display[1].cols, display[1].rows);
				std::printf("Depth: %s %dx%d \n", tyDepth.c_str(), imageZedDepth.cols, imageZedDepth.rows);
				onlyFirstTime = true;
			}*/


		}

	}

	virtual cv::Mat getLeftRGB()
	{
		return display[0];
	}
	virtual cv::Mat getRightRGB()
	{
		return display[1];
	}

	virtual cv::Mat getDepthMap()
	{

		return imageZedDepth;
	}
	/*static void callbackButton(int state, void *userdata)
	{

	}*/
	cv::Mat calibFrame(cv::Mat pframe, cv::Mat M, cv::Mat D, cv::Mat R, cv::Mat P)
	{
		cv::Size imageSize = pframe.size();
		cv::Mat rmap[1][1];
		cv::Mat img1r;
		cv::Mat map11, map12;

		initUndistortRectifyMap(M, D, R, P, imageSize, CV_16SC2, map11, map12);
		remap(pframe, img1r, map11, map12, cv::INTER_CUBIC);
		normalize(img1r, img1r, 0, 255, CV_MINMAX, CV_8U);
		//normalize(vzor, vzor,   0, 255, CV_MINMAX, CV_8U);

		return img1r;
	}

	/*
	* Conversion function between sl::Mat and cv::Mat
	*/
/*
	cv::Mat slMat2cvMat(sl::Mat& input) {
		// Mapping between MAT_TYPE and CV_TYPE
		int cv_type = -1;
		switch (input.getDataType()) {
		case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
		case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
		case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
		case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
		case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
		case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
		case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
		case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
		default: break;
		}

		// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
		// cv::Mat and sl::Mat will share a single memory structure
		return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
	}
*/
	/*
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
	}*/
};
