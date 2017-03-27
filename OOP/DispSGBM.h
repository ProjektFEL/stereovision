
#include "IDisparity.h"

class DispSGBM : public IDisparity{
private:
	int vmin, vmax, smin, mdip, ndip, sp1, sp2; // premenne pre stereosgbm
	int dmd, pfc, sur, sws, ssr, sm, bsiz;          //  premenne pre stereosgbm
	int alpha, beta;
	Mat disparity, depth;
	Mat imgLeft, imgRight;
	VideoCapture cap1;
	//string filename = "0";
	
public:
	DispSGBM()
	{
		vmin = 16, vmax = 3, smin = 0, mdip = 39, ndip = 10, sp1 = 655, sp2 = 30,
		dmd = 99, pfc = 0, sur = 17, sws = 10, ssr = 10, sm = 10, bsiz = 3;
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
		createTrackbar("dmd", "StereoBM control", &dmd, 99, 0);
		createTrackbar("bsiz", "StereoBM control", &bsiz, 99, 0);
		createTrackbar("sp1", "StereoBM control", &sp1, 1000, 0);
		createTrackbar("sp2", "StereoBM control", &sp2, 5000, 0);
		createTrackbar("pfc", "StereoBM control", &pfc, 200, 0);
		createTrackbar("sur", "StereoBM control", &sur, 30, 0);
		createTrackbar("sws", "StereoBM control", &sws, 200, 0);
		createTrackbar("ssr", "StereoBM control", &ssr, 30, 0);



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
			sgbm->setDisp12MaxDiff(dmd - 21);
			sgbm->setUniquenessRatio(sur - 15);
			sgbm->setSpeckleWindowSize(sws - 10);
			sgbm->setSpeckleRange(ssr - 10);

			sgbm->compute(frameLeft, frameRight, disparity);
			normalize(disparity, disparity, alpha, beta, CV_MINMAX, CV_8U);
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