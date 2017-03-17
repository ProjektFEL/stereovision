#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <string>
#include <iostream>

#include "structures.h"

#ifndef LINEASSIST_H
#define LINEASSIST_H

using namespace cv;
using namespace std;

class LineAssist
{

private:

	Mat frame;

public:

	LineAssist();
	LineAssist(VideoCapture capture);
	virtual ~LineAssist();

	Mat getFrame();
	void setFrame(Mat setFrame);
	LineAssist laneDetect(LineAssist laneAssistFrame, laneAssistKoef laneAssist);

};

#endif

