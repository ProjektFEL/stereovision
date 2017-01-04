#ifndef GUI_H
#define GUI_H

#include "opencv2/ximgproc/disparity_filter.hpp"
#include "opencv2/ximgproc/edge_filter.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include <string>
#include <iostream>
#include "DisparityAlghoritm.h"

using namespace cv;
using namespace std;

class Gui {
private:
	DisparityAlghoritm disparityAlghoritm;
public:
	Gui();
	Gui(DisparityAlghoritm disparityAlghoritm);
	~Gui();
	void showDisparitySettings();
	void showFiltersSettings();
};
#endif