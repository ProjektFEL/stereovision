
#include "IDisparity.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

class DispSGBM : public IDisparity{
private:
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
		vmin = 16, vmax = 3, smin = 0, mdip = 39, ndip = 10, sp1 = 655, sp2 = 30, pfc = 0, sm = 10, bsiz = 3;
//		dmd = 99, sur = 17, sws = 10, ssr = 10;
		alpha = 0, beta = 300;

		cvNamedWindow("StereoBM control", CV_WINDOW_AUTOSIZE);
		resizeWindow("StereoBM control", 400, 500);
		moveWindow("StereoBM control", 1000, 0);
	}

	~DispSGBM()
	{}


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



		//Mat frame;
		//cap0 >> imgLeft;
		//cap1.read(imgRight);

			if (vmax == 0) vmax = 1;
			Ptr<StereoSGBM> sgbm = StereoSGBM::create(vmin + 1, 16 * vmax, 2 * smin + 1);
			sgbm->setMinDisparity(mdip - 50);
			sgbm->setBlockSize(bsiz + 1);
			sgbm->setP1(sp1 - 10);
			sgbm->setP2(sp2 - 10);
			sgbm->setPreFilterCap(pfc - 10);
			sgbm->setMode(StereoSGBM::MODE_HH);


			sgbm->compute(frameLeft, frameRight, disparity16U);
			normalize(disparity16U, disparity, alpha, beta, CV_MINMAX, CV_8U);
			erode(disparity, disparity, getStructuringElement(MORPH_RECT, Size(3, 3)));
			dilate(disparity, disparity, getStructuringElement(MORPH_RECT, Size(3, 3)));
			//disparity.convertTo(disparity, CV_8U, 255);
			/*imshow("edges", frameLeft);
			waitKey(1);*/
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