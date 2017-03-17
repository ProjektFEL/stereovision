#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <string>
#include <iostream>


#include "structures.h"

#ifndef CAPTURE_H
#define CAPTURE_H

using namespace cv;
using namespace std;

class Capture
{
    public:

        Capture();
        Capture(VideoCapture capture);
        virtual ~Capture();

        Mat getFrame();
        void setFrame(Mat setFrame);
        Capture filterFrame(Capture frame, filterKoef filter);
        Capture resFilterFrame(Capture capture, filterKoef filter );
        Capture calibFrame(Capture capture,Mat M, Mat D, Mat R, Mat P);
        Capture stereoCalc(Capture captureL, Capture captureR, disparityKoef disparity);
        Capture birdViewTransform(Capture lFrameL,laneAssistKoef laneAssist);
        Capture laneDetect(Capture laneAssistFrame, laneAssistKoef laneAssist);

    protected:

    Mat frame;

    private:

};

#endif
