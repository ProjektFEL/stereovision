
#include "IDisparity.h"

class DispSGBM : public IDisparity{
private:
	int vmin, vmax, smin, mdip, ndip, sp1, sp2; // premenne pre stereosgbm
	int dmd, pfc, sur, sws, ssr, sm, bsiz;          //  premenne pre stereosgbm
	Mat disparity, depth;
	Mat imgLeft, imgRight;
	VideoCapture cap1;
	//string filename = "0";
	
public:
	DispSGBM()
	{
		vmin = 1, vmax = 3, smin = 0, mdip = 43, ndip = 10, sp1 = 266, sp2 = 4782,
		dmd = 99, pfc = 5, sur = 17, sws = 10, ssr = 10, sm = 10, bsiz = 4;
		
	}

	~DispSGBM()
	{}


	virtual void calculate(Mat frameLeft, Mat frameRight)
	{

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
			disparity.convertTo(disparity, CV_8U, 255);
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