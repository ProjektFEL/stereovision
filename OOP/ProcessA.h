
#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"



class ProcessA : public IProcess {
private: int ptX1 = 80, ptX2 = 80, ptY1 = 145, ptY2 = 145, gray = 15;
		 int theta1 = 56, rho1 = 0, theta2 = 19, rho2 = 0, max_lenght1 = 24, max_lenght2 = 27, edmin = 2, edmax = 255;


		 Mat birdViewCapture;
public:
	 
	ProcessA(){
	}

	void process(Mat frameLeft, Mat frameRight){

		 cvtColor(frameLeft, frameLeft, COLOR_BGR2GRAY);
		 cvtColor(frameRight, frameRight, COLOR_BGR2GRAY);

		 Point2f src[4], dst[4];
		 src[0].x = ptX1;
		 src[0].y = ptY1;
		 src[1].x = frameLeft.cols - ptX2;
		 src[1].y = ptY2;
		 src[2].x = frameLeft.cols;
		 src[2].y = frameLeft.rows;
		 src[3].x = 0;
		 src[3].y = frameLeft.rows;

		 dst[0].x = 0;
		 dst[0].y = 0;
		 dst[1].x = 320 - 1;
		 dst[1].y = 0;
		 dst[2].x = 320 - 1;
		 dst[2].y = 240 - 1;
		 dst[3].x = 0;
		 dst[3].y = 240 - 1;

		 Mat undistorted = Mat(cvSize(320, 240), CV_8UC1);
		 warpPerspective(frameLeft, frameLeft, getPerspectiveTransform(src, dst), cvSize(320, 240));

		 frameLeft.copyTo(birdViewCapture);
		 
	 }

	

	 Mat getFrame(){
		 return birdViewCapture;
	 }

	~ProcessA()
	{}
	 Mat getObject() {
		 return birdViewCapture;
	 }

};