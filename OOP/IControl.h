
#include <stdio.h>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


using namespace std;

#ifndef IControl_H
#define IControl_H
template <typename L,typename S>
class IControl {
public:
	virtual ~IControl(){};
	virtual void process(cv::Mat object, L line, []S signs) = 0;
	virtual int getWheel() = 0;
	virtual bool getDirection() = 0;
	virtual int getStrength() = 0;
	virtual int getBrake() = 0;

};
#endif