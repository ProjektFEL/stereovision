
#include "ICapture.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>
#include "sl/Camera.hpp"
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include <sl/Mesh.hpp>
#include <sl/types.hpp>


using namespace boost;
using namespace std;

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
	RuntimeParameters runtime_parameters;
	// Create a ZED camera object
	Camera zed;

	

	// parameters are videoFile(path to file) 
	CapZED3D(string pPath1, string pPath2)
	{
		isInputCamera = false;
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
		}
		catch (...)
		{
			cout << "Error in parsing Filter in CaptureZen3D!" << endl;
		}
		cvNamedWindow("InputFilter control", CV_WINDOW_AUTOSIZE);
		cv::resizeWindow("InputFilter control", 400, 500);
		cv::moveWindow("InputFilter control", 800, 0);
		//createButton("InputFilter controlaaa", callbackButton, NULL, QT_CHECKBOX, 0);
		
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
		
		size = cv::Size(320, 240);
		
		cvNamedWindow("InputFilter control", CV_WINDOW_AUTOSIZE);
		cv::resizeWindow("InputFilter control", 400, 500);
		cv::moveWindow("InputFilter control", 800, 0);
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
			
			

			// Set configuration parameters
			InitParameters init_params;

			init_params.camera_resolution = RESOLUTION_VGA;
			init_params.depth_mode = DEPTH_MODE_PERFORMANCE; // Use PERFORMANCE depth mode
			init_params.coordinate_units = UNIT_MILLIMETER; // Use millimeter units (for depth measurements)

															// Open the camera
			ERROR_CODE err = zed.open(init_params);
			if (err != SUCCESS)
				exit(-1);

			// Set runtime parameters after opening the camera
			
			runtime_parameters.sensing_mode = SENSING_MODE_STANDARD; // Use STANDARD sensing mode
			
		}
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

		if (isOpenmpEnabled == 0)
		{
			capture[0].read(frame[0]);
			capture[1].read(frame[1]);

			erode(frame[0], frame[0], getStructuringElement(cv::MORPH_RECT, cv::Size(erodeSizeW, erodeSizeH)));
			dilate(frame[0], frame[0], getStructuringElement(cv::MORPH_RECT, cv::Size(dilateSizeW, dilateSizeH)));
			erode(frame[1], frame[1], getStructuringElement(cv::MORPH_RECT, cv::Size(erodeSizeW, erodeSizeH)));
			dilate(frame[1], frame[1], getStructuringElement(cv::MORPH_RECT, cv::Size(dilateSizeW, dilateSizeH)));
			resize(frame[0], frame[0], size);
			resize(frame[1], frame[1], size);
			//frameGray[TID].convertTo(frame[TID], CV_8UC3, 1);
			bilateralFilter(frame[0], display[0], sigmaColor, sigmaSpace, borderType);
			bilateralFilter(frame[1], display[1], sigmaColor, sigmaSpace, borderType);
		}

		if (isCameraZed == 0 || isOpenmpEnabled == 1) {
		

		
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
			}

		}
		else if (isCameraZed == 1){

			

			sl::Mat imageLeft, imageRight, depth, point_cloud, imageDepth;
			float distance;

			if (zed.grab(runtime_parameters) == SUCCESS) {
				// Retrieve left image
				zed.retrieveImage(imageLeft, VIEW_LEFT);
				zed.retrieveImage(imageRight, VIEW_RIGHT);
				zed.retrieveImage(imageDepth, VIEW_DEPTH);
				// Retrieve depth map. Depth is aligned on the left image
				zed.retrieveMeasure(depth, MEASURE_DEPTH);
				// Retrieve colored point cloud. Point cloud is aligned on the left image.
				zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA);

				// Get and print distance value in mm at the center of the image
				// We measure the distance camera - object using Euclidean distance
				int x = imageLeft.getWidth() / 2;
				int y = imageLeft.getHeight() / 2;
				sl::float4 point_cloud_value;
				point_cloud.getValue(x, y, &point_cloud_value);

				 distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
				printf("Distance to Camera at (%d, %d): %f mm\n", x, y, distance);
				
			}

			slMat2cvMat(imageLeft).copyTo(display[0]);
			slMat2cvMat(imageRight).copyTo(display[1]);
			slMat2cvMat(imageDepth).copyTo(imageZedDepth);
			

			
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

	/**
	* Conversion function between sl::Mat and cv::Mat
	**/
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

	

};