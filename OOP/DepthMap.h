#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <string>
#include <iostream>
#include "structures.h"

#ifndef DEPTHMAP_H
#define DEPTHMAP_H

class DepthMap
{
private:
	Mat frame;

public:

	DepthMap();
	DepthMap(VideoCapture capture);
	virtual ~DepthMap();

	Mat getFrame();
	void setFrame(Mat setFrame);
	DepthMap filterFrame(DepthMap frame, filterKoef filter);
	DepthMap stereoCalc(DepthMap captureL, DepthMap captureR, disparityKoef disparity);
	DepthMap resFilterFrame(DepthMap capture, filterKoef filter);

};

#endif

