
#include "IDisparity.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <thread> 
#include <chrono>

class DispSGBM : public IDisparity{
private:
	//std::thread *first;
	thread *first, *second, *third, *fourth;

	int vmin, vmax, smin, mdip, ndip, sp1, sp2, pfc, sm, bsiz; // premenne pre stereosgbm
	//int dmd, sur, sws, ssr;          //  premenne pre stereosgbm
	int alpha, beta;
	Mat disparity16U,disparity, depth;
	Mat imgLeft, imgRight;
	VideoCapture cap1;
	//string filename = "0";
	
public:
	DispSGBM()
	{
		//first = new thread();
		vmin = 16, vmax = 3, smin = 0, mdip = 39, ndip = 10, sp1 = 655, sp2 = 30, pfc = 0, sm = 10, bsiz = 3;
//		dmd = 99, sur = 17, sws = 10, ssr = 10;
		alpha = 0, beta = 300;

		cvNamedWindow("StereoBM control", CV_WINDOW_AUTOSIZE);
		resizeWindow("StereoBM control", 400, 500);
		moveWindow("StereoBM control", 1000, 0);
	}

	~DispSGBM()
	{
		/*
		first->~thread();
		second->~thread();
		third->~thread();
		fourth->~thread();
		*/
	}


	virtual void calculate(Mat frameLeft, Mat frameRight)
	{
		createTrackbar("Vmin", "StereoBM control", &vmin, 99, 0);
		createTrackbar("Vmax", "StereoBM control", &vmax, 15, 0);
		createTrackbar("Smin", "StereoBM control", &smin, 30, 0);
		createTrackbar("mdip", "StereoBM control", &mdip, 99, 0);
		//createTrackbar("dmd", "StereoBM control", &dmd, 99, 0);
		createTrackbar("bsiz", "StereoBM control", &bsiz, 99, 0);
		createTrackbar("sp1", "StereoBM control", &sp1, 1000, 0);
		createTrackbar("sp2", "StereoBM control", &sp2, 5000, 0);
		createTrackbar("pfc", "StereoBM control", &pfc, 200, 0);
		//createTrackbar("sur", "StereoBM control", &sur, 30, 0);
		//createTrackbar("sws", "StereoBM control", &sws, 200, 0);
		//createTrackbar("ssr", "StereoBM control", &ssr, 30, 0);
		createTrackbar("alpha", "StereoBM control", &alpha, 300, 0);
		createTrackbar("beta", "StereoBM control", &beta, 300, 0);

		/*bilateralFilter(frameLeft, frameLeft, 9, 75, 75);
		bilateralFilter(frameRight, frameRight, 9, 75, 75);*/

		//Mat frame;
		//cap0 >> imgLeft;
		//cap1.read(imgRight);

		//auto start = chrono::high_resolution_clock::now();
			if (vmax == 0) vmax = 1;

			first = new thread(&DispSGBM::calculateSGBM, this, frameLeft, frameRight);
			//first->join();
			second = new thread(&DispSGBM::normalizeThread, this);
			//second->join();
			first = new thread(&DispSGBM::erodeThread, this);
			//third->join();
			second = new thread(&DispSGBM::dilateThread, DispSGBM());
			//fourth->join();
			/*
			calculateSGBM(frameLeft, frameRight);
			normalizeThread();
			erodeThread();
			dilateThread();
			*/

			//auto end = chrono::high_resolution_clock::now();
			//auto dur = chrono::duration_cast<std::chrono::duration<float>>(end - start);
			//std::cout << "Disparita:" << dur.count() << " ms" << endl;
			//normalize(disparity16U, disparity, alpha, beta, CV_MINMAX,CV_8UC3);
			//erode(disparity, disparity, getStructuringElement(MORPH_RECT, Size(3, 3)));
			//dilate(disparity, disparity, getStructuringElement(MORPH_RECT, Size(3, 3)));

			//disparity.convertTo(disparity, CV_8U, 255);
			/*imshow("edges", frameLeft);
			waitKey(1);*/
	}
	
	void normalizeThread(){
		normalize(disparity16U, disparity, alpha, beta, CV_MINMAX, CV_8UC3);
		//cout << "vlakno1" << endl;
	}
	/*
	std::thread spawn() {
		return{ normalizeThread };
	}
	*/

	void calculateSGBM(Mat frameLeft, Mat frameRight)
	{
		Ptr<StereoSGBM> sgbm = StereoSGBM::create(vmin + 1, 16 * vmax, 2 * smin + 1);
		sgbm->setMinDisparity(mdip - 50);
		sgbm->setBlockSize(bsiz + 1);
		sgbm->setP1(sp1 - 10);
		sgbm->setP2(sp2 - 10);
		sgbm->setPreFilterCap(pfc - 10);
		sgbm->setMode(StereoSGBM::MODE_HH);


		sgbm->compute(frameLeft, frameRight, disparity16U);
		double min;
		double max;
		cv::minMaxIdx(disparity16U, &min, &max);
		cv::Mat adjMap;
		cv::convertScaleAbs(disparity16U, disparity, 255 / max);
	}


	void erodeThread(){
		erode(disparity, disparity, getStructuringElement(MORPH_RECT, Size(3, 3)));
		//cout << "vlakno2" << endl;
	}

	void dilateThread(){
		dilate(disparity, disparity, getStructuringElement(MORPH_RECT, Size(3, 3)));
		//cout << "vlakno3" << endl;
	}

	Mat getLeft()
	{
		return imgLeft;
	}
	
	Mat getRight()
	{
		return imgRight;
	}
	
	virtual Mat getDisparity()
	{
		return disparity;
	}

	virtual Mat getDepth()
	{
		return depth;
	}

};