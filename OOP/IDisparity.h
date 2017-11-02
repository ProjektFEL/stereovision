
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

#ifndef IDisparity_H
#define IDisparity_H
class IDisparity {
public:
	virtual ~IDisparity(){};
	virtual thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight) = 0;
	virtual void calculate(cv::Mat frameLeft, cv::Mat frameRight) = 0;
	virtual void work(cv::Mat frameLeft, cv::Mat frameRight) = 0;
	virtual cv::Mat getDisparity() = 0;
	virtual cv::Mat getDepth() = 0;
	virtual cv::Mat getLeft() = 0;
	virtual cv::Mat getRight() = 0;
};
#endif