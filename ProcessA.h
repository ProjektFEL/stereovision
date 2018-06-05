
#include "IProcess.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

using namespace std;

class ProcessA : public IProcess {
private:

	cv::Mat copyLeft, copyRight, copyDisparity, diffBackground;

public:
	ProcessA(){

	}

	~ProcessA()
	{}

thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight) {
		// z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		// z->unlock();

	}


	void work(cv::Mat frameLeft, cv::Mat frameRight){
		for (int i = 1; i <= 100; i++)
		{
			cout << "ProcessA: " << i << endl;
		}
	}


	void process(cv::Mat disparity, cv::Mat backgroundImage){

    cv::Mat diffImage, originalImage;
    cv::absdiff(backgroundImage,disparity,diffImage);
    cv::Mat foregroundMask = cv::Mat::zeros(diffImage.rows, diffImage.cols, CV_8UC1);
	disparity.copyTo(originalImage);
    int threshold = 40;
    float dist;
    for(int j=0; j<diffImage.rows; ++j)
    {
        for(int i=0; i<diffImage.cols; ++i)
        {
            cv::Scalar pix = diffImage.at<uchar>(j,i);
            dist = (pix[0]*pix[0] + pix[1]*pix[1] + pix[2]*pix[2] + pix[3]*pix[3] + pix[4]*pix[4] + pix[5]*pix[5]);
            dist = sqrt(dist);
            if(dist>threshold)
            {
                foregroundMask.at<unsigned char>(j,i) = 255;
            }
        }
    }
	cv::Mat blank;
    cv::inRange(foregroundMask, 100,255,foregroundMask );
    originalImage.copyTo(blank,foregroundMask);
	blank.copyTo(diffBackground);



	 }

	cv::Mat getFrame(){
		 return diffBackground;
	 }

	 // este prerobit tuto funkciu, zla navratova hodnota
	cv::Mat getObject() {
			 return copyDisparity;
	 }
	 double getAngle(){
	return 0;
	}

	cv::Mat getDisparity(){
		 return diffBackground;
	 }

};
