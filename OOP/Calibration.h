#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <string>
#include <iostream>
#include "structures.h"

#ifndef CALIBRATION_H
#define CALIBRATION_H

class Calibration
{
private:

	Mat frame;

public:

	Calibration();
	Calibration(VideoCapture capture);
	virtual ~Calibration();

	Mat getFrame();
	void setFrame(Mat setFrame);

	Calibration calibFrame(Calibration capture, Mat M, Mat D, Mat R, Mat P);
	Calibration birdViewTransform(Calibration lFrameL, laneAssistKoef laneAssist);

};

#endif

