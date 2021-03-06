
#include "IProcess.h"
#include "IPM.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

class ProcessIPM : public IProcess {
private:
	thread *t;
	cv::Mat copyLeft, copyRight, copyDisparity;
	cv::Mat frameIpm, frameGray, frameOut, frameDrawedLines;
	int param1Y, param1X, param2Y, param2X, param3Y, param3X, param4Y, param4X;
	int aErode, aDilate;
	bool nastavovac, isOutputOneFrame;

public:
	ProcessIPM(){
		cvNamedWindow("InverseMapping_Control", CV_WINDOW_AUTOSIZE);
		cv::resizeWindow("InverseMapping_Control", 400, 500);
		cv::moveWindow("InverseMapping_Control", 800, 300);
		param1X = 0, param1Y = 0, param2X = 0, param2Y = 0, param3X = 0, param3Y = 0, param4X = 0, param4Y = 0;
		aErode = 3, aDilate = 3;
		nastavovac = true;
		isOutputOneFrame = true;
	}

	~ProcessIPM()
	{}

	thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight)
	{
		z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		z->unlock();
		t = new thread(&ProcessIPM::process, this, copyLeft, copyRight);
		return t;
	}

	void work(cv::Mat frameLeft, cv::Mat frameRight){
		for (int i = 1; i <= 100; i++)
		{
			cout << "ProcessIPM: " << i << endl;
		}
	}

	void process(cv::Mat frameLeft, cv::Mat frameRight){
		if (!isOutputOneFrame)
		frameLeft.copyTo(frameDrawedLines);

		// nastavi na zakaldne hodnoty
		if (nastavovac){
			param1X=30;
			param1Y = frameLeft.rows;
			param2X = frameLeft.cols;
			param2Y = frameLeft.rows;
			param3X = 320;
			param3Y = 180;
			param4X = 190;
			param4Y = 180;
			nastavovac = false;
		}
		cvCreateTrackbar("param1X", "InverseMapping_Control", &param1X, frameLeft.cols);
		cvCreateTrackbar("param1Y", "InverseMapping_Control", &param1Y, frameLeft.rows);
		cvCreateTrackbar("param2X", "InverseMapping_Control", &param2X, frameLeft.cols);
		cvCreateTrackbar("param2Y", "InverseMapping_Control", &param2Y, frameLeft.rows);
		cvCreateTrackbar("param3X", "InverseMapping_Control", &param3X, frameLeft.cols);
		cvCreateTrackbar("param3Y", "InverseMapping_Control", &param3Y, frameLeft.rows);
		cvCreateTrackbar("param4X", "InverseMapping_Control", &param4X, frameLeft.cols);
		cvCreateTrackbar("param4Y", "InverseMapping_Control", &param4Y, frameLeft.rows);
		cvCreateTrackbar("Erode", "InverseMapping_Control", &aErode, 10);
		cvCreateTrackbar("Dilate", "InverseMapping_Control", &aDilate, 10);

		// body na obrazku, ktore su potom vstupom na IPM
		vector<cv::Point2f> origPoints;
		origPoints.push_back(cv::Point2f(param1X, param1Y));
		origPoints.push_back(cv::Point2f(param2X, param2Y));
		origPoints.push_back(cv::Point2f(param3X, param3Y));
		origPoints.push_back(cv::Point2f(param4X, param4Y));

		vector<cv::Point2f> dstPoints;
		dstPoints.push_back(cv::Point2f(0, frameLeft.rows));
		dstPoints.push_back(cv::Point2f(frameLeft.cols, frameLeft.rows));
		dstPoints.push_back(cv::Point2f(frameLeft.cols, 0));
		dstPoints.push_back(cv::Point2f(0, 0));

		IPM ipm(cv::Size(frameLeft.cols, frameLeft.rows), cv::Size(frameLeft.cols, frameLeft.rows), origPoints, dstPoints);
		ipm.applyHomography(frameLeft, frameOut);

		//ak ma byt vystup jeden premapovany frame, treba prenastavit boolen premennu isOutputOneFrame na true
		if (!isOutputOneFrame){
		ipm.drawPoints(origPoints, frameDrawedLines);
		cv::Mat frameImpout(frameOut.rows, (frameOut.cols + frameDrawedLines.cols), CV_32F);
		hconcat(frameOut, frameDrawedLines, frameImpout);  // dva frame vedla seba
		frameIpm = frameImpout;
		}
		else
		{
			erode(frameIpm, frameIpm, getStructuringElement(cv::MORPH_RECT, cv::Size(aErode, aErode)));
			dilate(frameIpm, frameIpm, getStructuringElement(cv::MORPH_RECT, cv::Size(aDilate, aDilate)));
			frameIpm = frameOut;
		}
		
	}

	cv::Mat getFrame(){
		return frameIpm;
	}

	// este prerobit tuto funkciu, zla navratova hodnota
	cv::Mat getObject() {
		return frameIpm;
	}
};