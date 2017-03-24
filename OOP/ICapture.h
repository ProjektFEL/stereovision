
#include <stdio.h>
#include <string>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


using namespace cv;
using namespace std;

#ifndef ICapture_H
#define ICapture_H
class ICapture {
public:
	virtual ~ICapture(){};
	virtual void process() = 0;
	virtual Mat getLeftRGB() = 0;
	virtual Mat getRightRGB() = 0;
	virtual Mat getDepthMap() = 0;
};
#endif