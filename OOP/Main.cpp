#include "Interface.h"
#include <stdio.h>
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc/edge_filter.hpp"
#include "DisparityMap.h"

int main() {
	DisparityAlghoritm dispAlg = DisparityAlghoritm();
	Gui gui = Gui(dispAlg);

	DisparityMap dispMap = DisparityMap("C:\\Users\\Gamer\\Desktop\\3D kamera\\video.MTS", dispAlg, gui);
	int choose=0;
	do
	{
		choose=dispMap.calculateDepthOneCamera();
	} while (choose == 1);
	return 0;
};