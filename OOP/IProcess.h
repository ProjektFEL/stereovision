
#include <stdio.h>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


using namespace cv;
using namespace std;

//lines L
//sign S

template <typename L, typename S>
class IProcess {
public:
	virtual ~IProcess(){};
	virtual void process(Mat rgb, Mat depthMap) = 0;
	virtual Mat getObject() = 0;
	virtual <L> getLine() = 0;
	virtual <S>[] getSign() = 0;
};