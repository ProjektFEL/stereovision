
#include <stdio.h>
#include <string>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"



using namespace std;

#ifndef ICapture_H
#define ICapture_H
class ICapture {
public:
	virtual ~ICapture(){};
	virtual void process() = 0;
	virtual cv::Mat getLeftRGB() = 0;
	virtual cv::Mat getRightRGB() = 0;
	virtual cv::Mat getDepthMap() = 0;
};
#endif