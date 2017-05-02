
#include <stdio.h>
#include <string>
#include <iostream>
#include <thread> 
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


using namespace cv;
using namespace std;

//lines L
//sign S

#ifndef IProcess_H
#define IProcess_H
class IProcess {
public:
	
	IProcess(){};
	virtual ~IProcess(){};
	virtual thread* run(mutex* z, Mat frameLeft, Mat frameRight) = 0;
	virtual void process(Mat frameLeft, Mat frameRight) = 0;
	virtual void work(Mat frameLeft, Mat frameRight) = 0;
	virtual Mat getObject() = 0;
	virtual Mat getFrame()=0;
	
};
#endif