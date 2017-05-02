
#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost;

class ProcessC : public IProcess {
private: 
	thread *t;
	Mat copyLeft, copyRight;
	Mat imageROI,frame1,original, frameEroded,temp,img;
	int cny_threshold, cny_apertureSize, trsh_maxValue, trsh_blockSize, trsh_constant, trsh_constant_Compt;
	double hough_rho, hough_theta, hough_tresh, hough_minLength, hough_maxLine;
	int angleMaxLeft, angleMinLeft, angleMaxRight, angleMinRight;
	int angleMaxLeft_Compt, angleMinLeft_Compt, angleMaxRight_Compt, angleMinRight_Compt;
public:
	// konstruktor, nacitavanie atributov zo suboru
	ProcessC(){
		cny_threshold = 100;
		cny_apertureSize = 3;
		trsh_maxValue = 255;
		trsh_blockSize = 5;
		trsh_constant = 7;
		trsh_constant_Compt = 50;
		hough_rho = 1;
		hough_theta = CV_PI / 90;
		hough_tresh = 2;
		hough_minLength = 30;
		hough_maxLine = 3;
		angleMaxLeft= 0, angleMinLeft = 0, angleMaxRight= 0, angleMinRight = 0;

		cvNamedWindow("LaneDetect control C", CV_WINDOW_AUTOSIZE);
		resizeWindow("LaneDetect control C", 400, 500);
		moveWindow("LaneDetect control C", 1300, 0);
	}

	thread* run(mutex* z, Mat frameLeft, Mat frameRight){
		//z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		//z->unlock();
		t = new thread(&ProcessC::process, this, copyLeft, copyRight);
		return t;
	}

	void work(Mat frameLeft, Mat frameRight){
		for (int i = 1; i <= 100; i++)
		{
			cout << "ProcessC: " << i << endl;
		}
	}
	
	void process(Mat frameLeft,Mat frameRight){
		int divideFrame = 6;
		int frameColumns = frameLeft.cols;
		int frameRows = frameLeft.rows;
		
		cvCreateTrackbar("cny_threshold", "LaneDetect control C", &cny_threshold, 200);
		cvCreateTrackbar("cny_apertureSize", "LaneDetect control C", &cny_apertureSize, 10);
		cvCreateTrackbar("trsh_maxValue", "LaneDetect control C", &trsh_maxValue, 255);
		cvCreateTrackbar("trsh_blockSize", "LaneDetect control C", &trsh_blockSize, 7);
		cvCreateTrackbar("trsh_constant", "LaneDetect control C", &trsh_constant_Compt, 100);
		/*cvCreateTrackbar("hough_rho", "LaneDetect control C", &hough_rho, 10);*/
		//cvCreateTrackbar("hough_theta", "LaneDetect control C", &hough_theta, 180);
		//cvCreateTrackbar("hough_tresh", "LaneDetect control C", &hough_tresh, 10);
		//cvCreateTrackbar("hough_minLength", "LaneDetect control C", &hough_minLength, 50);
		//cvCreateTrackbar("hough_maxLine", "LaneDetect control C", &hough_maxLine, 50);
		cvCreateTrackbar("angle<MaxLeft", "LaneDetect control C", &angleMaxLeft_Compt, 720);
		cvCreateTrackbar("angle>MinLeft", "LaneDetect control C", &angleMinLeft_Compt, 720);
		cvCreateTrackbar("angle<MaxRight", "LaneDetect control C", &angleMaxRight_Compt, 720);
		cvCreateTrackbar("angle>MinRight", "LaneDetect control C", &angleMinRight_Compt, 720);

		frameLeft = frameLeft(Rect(frameColumns / divideFrame, 0, (frameColumns)-frameColumns / divideFrame, frameRows));
		frameLeft.copyTo(original);
		cvtColor(frameLeft, frameLeft, CV_BGR2GRAY);
		equalizeHist(frameLeft, frameLeft);
		if (trsh_blockSize < 3)trsh_blockSize = 3;
		if (trsh_blockSize > 7)trsh_blockSize = 7;
		if (trsh_blockSize < 7 && trsh_blockSize > 3)trsh_blockSize = 5;
		if (trsh_constant_Compt > 50)trsh_constant = 0 + (trsh_constant_Compt)-50;
		if (trsh_constant_Compt < 50)trsh_constant = 0 + (trsh_constant_Compt)-50;
		if (trsh_constant_Compt == 50)trsh_constant = 0;
		if (angleMaxLeft > 360)angleMaxLeft = 0 + (angleMaxLeft_Compt - 360);
		if (angleMaxLeft < 360)angleMaxLeft = 0 + (angleMaxLeft_Compt - 360);
		if (angleMinLeft > 360)angleMinLeft = 0 + (angleMinLeft_Compt - 360);
		if (angleMinLeft < 360)angleMinLeft = 0 + (angleMinLeft_Compt - 360);
		if (angleMaxRight > 360)angleMaxRight = 0 + (angleMaxRight_Compt - 360);
		if (angleMaxRight < 360)angleMaxRight = 0 + (angleMaxRight_Compt - 360);
		if (angleMinRight > 360)angleMinRight = 0 + (angleMinRight_Compt - 360);
		if (angleMinRight < 360)angleMinRight = 0 + (angleMinRight_Compt - 360);
		
		adaptiveThreshold(frameLeft, frameLeft, trsh_maxValue, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, trsh_blockSize, trsh_constant);
		
		
		Canny(frameLeft, frameLeft, cny_threshold, cny_threshold, cny_apertureSize);
		
		vector<Vec4i> hg_lines;
		vector<Vec4i> hg_lines_partition;
		HoughLinesP(frameLeft, hg_lines, hough_rho, hough_theta, hough_tresh, hough_minLength, hough_maxLine);
		Mat Blank(frameLeft.rows, frameLeft.cols, CV_8UC3, Scalar(0, 0, 0));
		Mat Blank2(Blank.rows, Blank.cols, CV_8UC3, Scalar(0, 0, 0));
		int w = frameLeft.rows;
		int h = frameLeft.cols;
		int center = h / 2;
		for (size_t i = 0; i < hg_lines.size(); i++){
			Vec4i l = hg_lines[i];

			Point p1, p2;
			p1 = Point(l[0], l[1]);
			p2 = Point(l[2], l[3]);
			//calculate angle in radian,  if you need it in degrees just do angle * 180 / PI
			float angle = (float) atan2(p1.y - p2.y, p1.x - p2.x);
			angle = (float) (angle * 180 / CV_PI);
			
			if (l[0]<h*2 / 3 && l[2]<h*2 / 3){
				
				/*if (angle < angleMaxLeft && angle > angleMinLeft ){*/
					
					line(original, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 255), 3);
					line(original, Point(l[2], l[3]), Point(l[2], l[3]), Scalar(0, 0, 255), 7);
					//line(original, Point(l[0], l[1]), Point(l[0], l[1]), Scalar(255, 0, 0), 5);
					
				/*}*/
			}
			if (l[0]>h / 3 && l[2]>h / 3){
				/*if (angle < angleMaxRight && angle > angleMinRight){*/
					line(original, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 0), 3);
					line(original, Point(l[2], l[3]), Point(l[2], l[3]), Scalar(0, 0, 255), 7);
					//line(original, Point(l[0], l[1]), Point(l[0], l[1]), Scalar(255, 0, 0), 5);
				/*}*/
			}
			/*erode(Blank, Blank, getStructuringElement(MORPH_RECT, Size(2, 2)));
			dilate(Blank, Blank, getStructuringElement(MORPH_RECT, Size(5, 5)));
			
			Canny(Blank, Blank, cny_threshold, cny_threshold, 3);
			HoughLinesP(Blank, hg_lines_partition, 1, CV_PI / 90, 2, 30, 3);
			
			for (size_t i = 0; i < hg_lines_partition.size(); i++){
				Vec4i l = hg_lines_partition[i];
				line(Blank2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3);
			}*/
		}
				
		frame1 = original;
	 }

	 Mat getFrame(){
		 return frame1;
	 }

	// este prerobit tuto funkciu, zla navratova hodnota
	 Mat getObject() {
		 return frame1;
	 }

	 ~ProcessC()
	 {}


};
