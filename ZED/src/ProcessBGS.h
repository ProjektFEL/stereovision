
#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

class ProcessBGS : public IProcess {
private:
	thread *t;
	cv::Mat copyLeft, copyRight, copyDisparity;
	cv::Mat  framePrekazky;
	int olda, oldb, oldc;
	cv::Mat  ObjectCoordinates;
	cv::String motion;

public:
	ProcessBGS(){
		int olda = 0, oldb = 0, oldc = 0;
		//backgroundImage = cv::imread("gradient.png", CV_LOAD_IMAGE_GRAYSCALE);


	}

	~ProcessBGS()
	{}


	thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight)
	{
		z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		z->unlock();
		t = new thread(&ProcessBGS::process, this, copyLeft, copyRight);
		return t;
	}

	void work(cv::Mat frameLeft, cv::Mat frameRight){
		for (int i = 1; i <= 100; i++)
		{
			cout << "ProcessBGS: " << i << endl;
		}
	}


	void process(cv::Mat disparity, cv::Mat backgroundImage){

		//Capture disparityFrame;
		//disparityFrame = disparityFrame.stereoCalc(dFrameL, dFrameR, disparity);
		//disparityFrame = disparityFrame.resFilterFrame(disparityFrame, filter);
		//Mat croped = disparity(Rect(30, 40, 270, 190)); //crop disparity, just calibrated ROI
		 // disparityFrame.setFrame(croped);
		//imshow("ranged", disparityFrame.getFrame());
		// imwrite("haha.png", disparityFrame.getFrame());
		cv::Mat diffImage;
		absdiff(backgroundImage, disparity, diffImage);

		cv::Mat foregroundMask = cv::Mat::zeros(diffImage.rows, diffImage.cols, CV_8UC1);

		float threshold = 30.0f;
		float dist;

		for (int j = 0; j<diffImage.rows; ++j)
		{
			for (int i = 0; i<diffImage.cols; ++i)
			{
				cv::Scalar pix = diffImage.at<uchar>(j, i);

				dist = (pix[0] * pix[0] + pix[1] * pix[1] + pix[2] * pix[2]);
				dist = sqrt(dist);

				if (dist>threshold)
				{
					foregroundMask.at<unsigned char>(j, i) = 255;
				}
			}
		}

		cv::Mat selected;
		inRange(foregroundMask, 100, 255, foregroundMask);
		//imshow("foregroundMask", foregroundMask);
		disparity.copyTo(selected, foregroundMask);

		erode(selected, selected, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);
		int a = 0, b = 0, c = 0;

		a = detect_object(selected, 0, 79); // ak sa zmeni rozlisenie, treba zmenit aj toto
		b = detect_object(selected, 80, 239);
		c = detect_object(selected, 240, 319);
		//cout << a << ":" << b << ":" << c << endl;
		if (a != olda || b != oldb || c != oldc)
		{
			motion = motionCar(a, b, c);
			ObjectCoordinates = (cv::Mat_<double>(3, 1) << a,b,c);
			olda = a;
			oldb = b;
			oldc = c;
		}

		/*putText(selected, motion, cv::Point2f(20, 20), cv::FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 255), 1, cv::LINE_AA);*/
		//imshow("raged", selected);
		framePrekazky = selected;

	 }

	 cv::Mat getFrame(){
		 return framePrekazky;
	 }

	 // este prerobit tuto funkciu, zla navratova hodnota
	 cv::Mat getObject() {
			 return ObjectCoordinates;
	 }

		 int detect_object(cv::Mat dispmap, uint x1, uint x2)
		 {
			 cv::Scalar valueD;
			 int i, j, y1 = 0, y2 = 239, pb = 0;
			 int prem = 0;
			 for (i = x1; i <= x2; i++)
			 {
				 for (j = y1; j <= y2; j++)
				 {
					 valueD = dispmap.at<uchar>(cv::Point(i, j));
					 if ((valueD.val[0]>50) && (valueD.val[0] <= 255))
					 {
						 pb++;
					 }
				 }
			 }

			 if (pb > 400)
			 {
				 return 1;
			 }
			 else
				 return 0;

		 }

		 string motionCar(int a, int b, int c)
		 {
			 string motion = "";
			 if ((a != 1) && (b != 1) && (c != 1))
			 {
				 motion = "bez prekazky";
			 }
			 if ((a != 1) && (b != 1) && (c == 1))
			 {
				 motion = "prekazka vpravo";
			 }
			 if ((a != 1) && (b == 1) && (c != 1))
			 {
				 motion = "prekazka v strede";
			 }
			 if ((a != 1) && (b == 1) && (c == 1))
			 {
				 motion = "prekazka vpravo a v strede";
			 }
			 if ((a == 1) && (b != 1) && (c != 1))
			 {
				 motion = "prekazka vlavo";
			 }
			 if ((a == 1) && (b != 1) && (c == 1))
			 {
				 motion = "prekazka vpravo a vlavo";
			 }
			 if ((a == 1) && (b == 1) && (c != 1))
			 {
				 motion = "prekazka v strede a vlavo";
			 }
			 if ((a == 1) && (b == 1) && (c == 1))
			 {
				 motion = "prekazka vpravo, vlavo a v strede";
			 }
			 return motion;
		 }
		 double getAngle(){
	return 0;
	}

};
