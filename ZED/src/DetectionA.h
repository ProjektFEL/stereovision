
#include "IDetection.h"
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


using namespace std;


class DetectionA : public IDetection {
private:
	thread *t;
	int a, b, c;
	uchar l1, l2, l3;
	string motion;
	cv::Mat copyLeft, copyRight, copyDisparity;
	cv::Mat close, medium, far;
public:
	DetectionA()
	{
		l1 = 90, l2 = 170, l3 = 255;
	};

	~DetectionA()
	{}

	thread* run(mutex* z, cv::Mat disparity, cv::Mat frameLeft, cv::Mat frameRight){
		z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		disparity.copyTo(copyDisparity);
		z->unlock();
		t = new thread(&DetectionA::calculate, this, copyDisparity, copyLeft, copyRight);
		return t;
	}

	void work(cv::Mat disparity, cv::Mat frameLeft, cv::Mat frameRight)
	{
		for (int i = 1; i <= 100; i++)
		{
			cout << "DetectionA: " << i << endl;
		}
	};

	int detect_object(cv::Mat disparity, uint x1, uint x2)
	{
		cv::Scalar valueD;
		uint i, j, y1 = 1, y2 = 239, pb = 0;
		int prem = 0;
		valueD = {0};

		for (i = x1; i <= x2; i++)
		{
			for (j = y1; j <= y2; j++)
			{
				if (disparity.empty() == true)
					return 1;
				else
				{
					
					valueD = disparity.at<uchar>(cv::Point(i,j)); //  chyba
					if ((valueD.val[0] > l1) && (valueD.val[0] < l2))
					{
						pb++;
					}
				}
			}
		}

		
		int dispWidth = disparity.rows;
		int dispHeight = disparity.cols;

		if (pb >= 70)
		{
			if ((i >= 30) && (i <= (dispWidth) / 3))prem = 1;

			if ((i >= (dispWidth / 3) + 1) && (i <= ((dispWidth) * 2 / 3)))prem = 2;

			if ((i >= ((dispWidth) * 2 / 3) + 1) && (i <= dispWidth - 1))prem = 3;
		}
		return prem;
	}

	string motionCar(int a, int b, int c)
	{
		if ((a != 1) && (b != 2) && (c != 3))
		{
			motion = "bez prekážky";
		}
		if ((a != 1) && (b != 2) && (c == 3))
		{
			motion = "prekážka vpravo";
		}
		if ((a != 1) && (b == 2) && (c != 3))
		{
			motion = "prekážka v strede";
		}
		if ((a != 1) && (b == 2) && (c == 3))
		{
			motion = "prekážka vpravo a v strede";
		}
		if ((a == 1) && (b != 2) && (c != 3))
		{
			motion = "prekázka vlavo";
		}
		if ((a == 1) && (b != 2) && (c == 3))
		{
			motion = "prekážka vpravo a vlavo";
		}
		if ((a == 1) && (b == 2) && (c != 3))
		{
			motion = "prekážka v strede a vlavo";
		}
		if ((a == 1) && (b == 2) && (c == 3))
		{
			motion = "prekážka vpravo, vlavo a v strede";
		}
		return motion;
	}


	void calculate(cv::Mat disparity, cv::Mat frameLeft, cv::Mat frameRight)
	{
		disparity.convertTo(disparity, CV_8UC1);
		cv::Mat backgroundImage;
		backgroundImage = cv::imread("disparita.png", CV_LOAD_IMAGE_GRAYSCALE);
		string tyDepth = type2str(backgroundImage.type());
		std::printf("Background: %s %dx%d \n", tyDepth.c_str(), backgroundImage.cols, backgroundImage.rows);
		cv::Mat diffImage;
		//diffImage.resize(backgroundImage.cols, backgroundImage.rows);
		cv::absdiff(backgroundImage, disparity, diffImage); //chyba
		cv::Mat foregroundMask = cv::Mat::zeros(diffImage.rows, diffImage.cols, CV_8UC1);

		float threshold = 20.0f;
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
		//imshow("foregroundMask",foregroundMask);
		disparity.copyTo(selected, foregroundMask);
		erode(selected, selected, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);
		int a = 0, b = 0, c = 0;
		inRange(selected, 110, 255, selected);
		cv::imshow("inRange", selected);

		/*
		int dispWidth = disparity.rows;
		int dispHeight = disparity.cols;
		
		disparity.convertTo(disparity, CV_8U);
		//cout << disparity << endl;
		//pracujeme s kopiami
			a = detect_object(disparity, 0, ((dispWidth)/3));
			b = detect_object(disparity, ((dispWidth) / 3)+1, ((dispWidth) *2/ 3));
			c = detect_object(disparity, ((dispWidth) *2/ 3) + 1, dispWidth-1);

			string prekazka = motionCar(a, b, c);
			
			
			inRange(disparity, cv::Scalar(0,0,0), cv::Scalar(l1, l1, l1), close);
			inRange(disparity, l1, l2, medium);
			inRange(disparity, l2, l3, far);

			cv::putText(close, prekazka, cv::Point(0, 20), cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
			*/
	};


	cv::Mat getCloseObj()
	{
		return close;
	};

	cv::Mat getMediumObj()
	{
		return medium;
	};

	cv::Mat getFarObj()
	{
		return far;
	};

	string type2str(int type) {
		string r;

		uchar depth = type & CV_MAT_DEPTH_MASK;
		uchar chans = 1 + (type >> CV_CN_SHIFT);

		switch (depth) {
		case CV_8U:  r = "8U"; break;
		case CV_8S:  r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:     r = "User"; break;
		}

		r += "C";
		r += (chans + '0');

		return r;
	}

};
