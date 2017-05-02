
#include "IDetection.h"
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"

using namespace cv;
using namespace std;


class DetectionA : public IDetection {
private:
	thread *t;
	int a, b, c;
	int l1, l2, l3;
	string motion;
	Mat copyLeft, copyRight, copyDisparity;
	Mat close, medium, far;
public:
	DetectionA()
	{
		l1 = 90, l2 = 255, l3 = 255;
	};

	~DetectionA()
	{}

	thread* run(mutex* z, Mat disparity, Mat frameLeft, Mat frameRight){
		z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		disparity.copyTo(copyDisparity);
		z->unlock();
		t = new thread(&DetectionA::calculate, this, copyDisparity, copyLeft, copyRight);
		return t;
	}

	void work(Mat disparity, Mat frameLeft, Mat frameRight)
	{
		for (int i = 1; i <= 100; i++)
		{
			cout << "DetectionA: " << i << endl;
		}
	};

	int detect_object(Mat disparity, uint x1, uint x2)
	{
		Scalar valueD;
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
					valueD = disparity.at<uint>(Point(i, j));
					if ((valueD.val[0] > l1) && (valueD.val[0] < l2))
					{
						pb++;
					}
				}
			}
		}

		if (pb >= 70)
		{
			if ((i >= 30) && (i <= 85))prem = 1;

			if ((i >= 86) && (i <= 200))prem = 2;

			if ((i >= 201) && (i <= 284))prem = 3;
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


	void calculate(Mat disparity, Mat frameLeft, Mat frameRight)
	{
			//pracujeme s kopiami
			a = detect_object(disparity, 40, 80);
			b = detect_object(disparity, 140, 199);
			c = detect_object(disparity, 203, 280);

			inRange(disparity, 0, l1, close);
			inRange(disparity, l1, l2, medium);
			inRange(disparity, l2, l3, far);
	};


	Mat getCloseObj()
	{
		return close;
	};

	Mat getMediumObj()
	{
		return medium;
	};

	Mat getFarObj()
	{
		return far;
	};

};
