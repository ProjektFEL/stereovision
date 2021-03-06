
#include "IDisparity.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

class SGBM : public IDisparity{
private:
	thread *t;
	cv::Mat copyLeft, copyRight;
	int vmin, vmax, smin, mdip, ndip, sp1, sp2, pfc, sm, bsiz; // premenne pre stereosgbm
	//int dmd, sur, sws, ssr;          //  premenne pre stereosgbm
	int alpha, beta;
	cv::Mat disparity16U,disparity, depth;
	cv::Mat imgLeft, imgRight;
	//string filename = "0";
	
public:
	SGBM()
	{
		vmin = 16, vmax = 3, smin = 0, mdip = 39, ndip = 10, sp1 = 655, sp2 = 30, pfc = 0, sm = 10, bsiz = 3;
//		dmd = 99, sur = 17, sws = 10, ssr = 10;
		alpha = 0, beta = 300;

		cvNamedWindow("StereoBM control", CV_WINDOW_AUTOSIZE);
		cv::resizeWindow("StereoBM control", 400, 500);
		cv::moveWindow("StereoBM control", 1000, 0);
	}

	~SGBM()
	{}


	thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight){
		z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		z->unlock();
		t = new thread(&SGBM::calculate, this, copyLeft, copyRight);
		return t;
	}

	void work(cv::Mat frameLeft, cv::Mat frameRight)
	{
		for (int i = 1; i <= 100; i++)
		{
			cout << "DispSGBM: " << i << endl;
		}
	}

	void calculate(cv::Mat frameLeft, cv::Mat frameRight)
	{
		cv::createTrackbar("Vmin", "StereoBM control", &vmin, 99, 0);
		cv::createTrackbar("Vmax", "StereoBM control", &vmax, 15, 0);
		cv::createTrackbar("Smin", "StereoBM control", &smin, 30, 0);
		cv::createTrackbar("mdip", "StereoBM control", &mdip, 99, 0);
		//createTrackbar("dmd", "StereoBM control", &dmd, 99, 0);
		cv::createTrackbar("bsiz", "StereoBM control", &bsiz, 99, 0);
		cv::createTrackbar("sp1", "StereoBM control", &sp1, 1000, 0);
		cv::createTrackbar("sp2", "StereoBM control", &sp2, 5000, 0);
		cv::createTrackbar("pfc", "StereoBM control", &pfc, 200, 0);
		//createTrackbar("sur", "StereoBM control", &sur, 30, 0);
		//createTrackbar("sws", "StereoBM control", &sws, 200, 0);
		//createTrackbar("ssr", "StereoBM control", &ssr, 30, 0);
		cv::createTrackbar("alpha", "StereoBM control", &alpha, 300, 0);
		cv::createTrackbar("beta", "StereoBM control", &beta, 300, 0);

		/*bilateralFilter(frameLeft, frameLeft, 9, 75, 75);
		bilateralFilter(frameRight, frameRight, 9, 75, 75);*/

		if (vmax == 0) vmax = 1;

		calculateSGBM(frameLeft, frameRight);
		normalize(disparity16U, disparity, alpha, beta, CV_MINMAX, CV_8UC3);
		erode(disparity, disparity, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
		dilate(disparity, disparity, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
	}

	void calculateSGBM(cv::Mat frameLeft, cv::Mat frameRight)
	{
		cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(vmin + 1, 16 * vmax, 2 * smin + 1);
		sgbm->setMinDisparity(mdip - 50);
		sgbm->setBlockSize(bsiz + 1);
		sgbm->setP1(sp1 - 10);
		sgbm->setP2(sp2 - 10);
		sgbm->setPreFilterCap(pfc - 10);
		sgbm->setMode(cv::StereoSGBM::MODE_HH);

		sgbm->compute(frameLeft, frameRight, disparity16U);
		double min;
		double max;
		cv::minMaxIdx(disparity16U, &min, &max);
		cv::Mat adjMap;
		cv::convertScaleAbs(disparity16U, disparity, 255 / max);
	}

	cv::Mat getLeft()
	{
		return imgLeft;
	}

	cv::Mat getRight()
	{
		return imgRight;
	}

	virtual cv::Mat getDisparity()
	{
		return disparity;
	}

	virtual cv::Mat getDepth()
	{
		return depth;
	}

};