#ifndef DISPARITYALGHORITM_H
#define DISPARITYALGHORITM_H

#include "opencv2/ximgproc/disparity_filter.hpp"
#include "opencv2/ximgproc/edge_filter.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

class DisparityAlghoritm {
private:
	int minDisparity,blockSize, p1, p2, dispMaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, vmin, vmax, smin;
public:
	DisparityAlghoritm();
	DisparityAlghoritm(int minDisparity, int blockSize, int p1, int p2, int dispMaxDiff, int preFilterCap, int uniquenessRatio, int speckleWindowSize,
		int speckleRange, int vMin, int vMax, int sMin);
	~DisparityAlghoritm();
	int getVMin();
	void setVMin(int vmin);
	int getVMax() const;
	void setVMax(int vmax);
	int getSMin() const;
	void setSMin(int smin);
	int getMinDisparity() const;
	void setMinDisparity(int minDisparity);
	int getBlockSize() const;
	void setBlockSize(int blockSize);
	int getP1() const;
	void setP1(int p1);
	int getP2() const;
	void setP2(int p2);
	int getDispMaxDiff() const;
	void setDispMaxDiff(int dispMaxDiff);
	int getPreFilterCap() const;
	void setPreFilterCap(int preFilterCap);
	int getUniquenessRatio() const;
	void setUniquenessRatio(int uniquenessRatio);
	int getSpeckleWindowSize() const;
	void setSpeckleWindowSize(int speckleWindowSize);	
	int getSpeckleRange() const;
	void setSpeckleRange(int speckleRange);
	Mat calculateSGBM(Mat leftImage, Mat rightImage, Mat leftDisparity, Mat rightDisparity);
};
#endif