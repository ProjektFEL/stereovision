
#include <stdio.h>
#include <string>
#include <iostream>
#include <thread> 
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


using namespace std;

#ifndef IDetection_H
#define IDetection_H

class IDetection {
public:
	virtual ~IDetection(){};
	virtual thread* run(mutex* z, cv::Mat disparity, cv::Mat frameLeft, cv::Mat frameRight) = 0;
	virtual void calculate(cv::Mat disparity, cv::Mat frameLeft, cv::Mat frameRight) = 0;
	virtual void work(cv::Mat disparity, cv::Mat frameLeft, cv::Mat frameRight) = 0;
	virtual cv::Mat getCloseObj() = 0;
	virtual cv::Mat getMediumObj() = 0;
	virtual cv::Mat getFarObj() = 0;
};
#endif