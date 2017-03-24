
#include <stdio.h>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"

using namespace cv;
using namespace std;

class IMotor {
public:
	virtual ~IMotor(){};
	virtual void process(int wheel, bool direction, int strength, bool brake) = 0;
};