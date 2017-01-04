#ifndef DISPARITYMAP_H
#define DISPARITYMAP_H

#include "opencv2/ximgproc/disparity_filter.hpp"
#include "opencv2/ximgproc/edge_filter.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include <string>
#include <iostream>
#include "DisparityAlghoritm.h"
#include "Gui.h"

using namespace cv;
using namespace std;

class DisparityMap {
private:
	DisparityAlghoritm disparityAlghoritm;
	VideoCapture rightCamera, leftCamera;
	Mat leftImage, rightImage, leftDisparity, rightDisparity, disparityMap, depthMap;
	Gui gui;
public:
	DisparityMap();
	DisparityMap(string Path, DisparityAlghoritm disparityAlghoritm, Gui gui);
	DisparityMap(string rigthPath, string leftPath, DisparityAlghoritm disparityAlghoritm, Gui gui);
	~DisparityMap();
	Mat getLeftImage() const;
	Mat getRightImage() const;
	Mat DisparityMap::resizeImage(Mat image, int Width, int Height);
	void setLeftImage(Mat leftImage);
	void setRightImage(Mat rightImage);
	void calculateOneCamera();
	void calculateTwoCameras();
	int calculateDepthOneCamera();
	int calculateDepthTwoCameras();
};
#endif