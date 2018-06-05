
#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost;
using namespace std;

class ProcessK : public IProcess {
private:
	thread *t;
	cv::Mat copyLeft, copyRight;
	cv::Mat frame1;
	
	cv::CascadeClassifier laneCascade;
	int minSize1, minSize2, maxSize1, maxSize2;
public:
cv::String cascade_name;// = "classlbp//znacky2.xml";
property_tree::ptree pt;  // citac .ini suborov
	// konstruktor, nacitavanie atributov zo suboru
	ProcessK(){
		minSize1 = 24;
		minSize2 = 24;
		maxSize1 = 96;
		maxSize2 = 96;
		//cvNamedWindow("Lane_Cascade_control", CV_WINDOW_AUTOSIZE);
		//cv::resizeWindow("Lane_Cascade_control", 400, 500);
		//cv::moveWindow("Lane_Cascade_control", 10, 0);
property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			
                        cascade_name = pt.get<string>("ProcessK.pathCascadeClass");

		}
		catch (...)
		{
			cout << "Error in parsing Filter in Application!" << endl;
		}
	}

	thread* run(mutex *z, cv::Mat frameLeft, cv::Mat frameRight){
		//z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		//z->unlock();
		t = new thread(&ProcessK::process, this, copyLeft, copyRight);
		return t;
	}

	void work(cv::Mat frameLeft, cv::Mat frameRight){
		for (int i = 1; i <= 100; i++)
		{
			cout << "ProcessK: " << i << endl;
		}
	}

	void process(cv::Mat frameLeft, cv::Mat frameRight){
		
cvCreateTrackbar("minSize1", "Lane_Cascade_control", &minSize1, 50);
		cvCreateTrackbar("minSize2", "Lane_Cascade_control", &minSize2, 50);
		cvCreateTrackbar("maxSize1", "Lane_Cascade_control", &maxSize1, 150);
		cvCreateTrackbar("maxSize2", "Lane_Cascade_control", &maxSize2, 150);

		if (!laneCascade.load(cascade_name)){
			printf("--(!)Error loading lane cascade, missing files");
				std::cout << cascade_name << endl;
				printf("\n");
		};
		if (laneCascade.load(cascade_name)){
			std::vector<cv::Rect> lanes;
			cv::Mat frame_gray;

			cvtColor(frameLeft, frame_gray, cv::COLOR_BGR2GRAY);
			equalizeHist(frame_gray, frame_gray);

			//-- Detect Lanes
			laneCascade.detectMultiScale(frame_gray, lanes, 1.1, 5, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(minSize1, minSize2), cv::Size(maxSize1, maxSize2));

			for (size_t i = 0; i < lanes.size(); i++)
			{
				/*Point center(lanes[i].x + lanes[i].width / 2, lanes[i].y + faces[i].height / 2);
				ellipse(frame, center, Size(faces[i].width / 2, faces[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);*/
				rectangle(frameLeft, lanes[i], cv::Scalar(0, 255, 0), 2, 8, 0);

				cv::Mat faceROI = frame_gray(lanes[i]);
				std::vector<cv::Rect> eyes;
			}
			frame1 = frameLeft;
		}
	 }

	cv::Mat getFrame(){
		 return frame1;
	 }

	// este prerobit tuto funkciu, zla navratova hodnota
	cv::Mat getObject() {
		 return frame1;
	 }

	 cv::Mat getDisparity(){
		 return frame1;
	 }
double getAngle(){
	return 0;
	}

	 ~ProcessK()
	 {}


};
