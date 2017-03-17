#include "LineAssist.h"
#include "structures.h"

using namespace cv;
using namespace std;

LineAssist::LineAssist()
{
}

LineAssist::LineAssist(VideoCapture capture)
{
	capture >> frame;
}
//getter
Mat LineAssist::getFrame()
{
	return frame;
}
//setter
void LineAssist::setFrame(Mat setFrame)
{
	frame = setFrame;
}

LineAssist::~LineAssist()
{
}



LineAssist LineAssist::laneDetect(LineAssist capture, laneAssistKoef laneAssist)
{
	Mat birdView = capture.getFrame();
	inRange(birdView, 0, laneAssist.gray, birdView);
	GaussianBlur(birdView, birdView, Size(3, 3), 0, 0, BORDER_DEFAULT);
	Mat sobelx64f, abs_sobelx64f, sobel8u;
	Sobel(birdView, sobelx64f, birdView.depth(), 1, 0, 3);
	convertScaleAbs(sobelx64f, abs_sobelx64f);

	Mat lineSobel;
	abs_sobelx64f.copyTo(lineSobel);
	cvtColor(lineSobel, lineSobel, CV_GRAY2RGB);

	Mat lSobel, rSobel;
	lSobel = abs_sobelx64f(Rect(0, 0, 160, 240));
	rSobel = abs_sobelx64f(Rect(160, 0, 160, 240));

	Point meanpt;
	int x = 0, y = 0;
	meanpt.x = 0;
	meanpt.y = 0;
	Mat line1, line2;

	vector<Vec2f> lines1;
	HoughLines(lSobel, lines1, laneAssist.rho1 + 1, CV_PI / laneAssist.theta1, laneAssist.max_lenght1, 0, 0);
	for (size_t i = lines1.size() - 2; i < lines1.size(); i++)
	{
		float rho1 = lines1[i][0], theta1 = lines1[i][1];
		Point pt1, pt2, pt;
		double a = cos(theta1), b = sin(theta1);
		double x0 = a*rho1, y0 = b*rho1;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		pt.x = lSobel.cols - 1000;
		pt.y = -1000;
		line(lineSobel, pt1, pt2, Scalar(0, 255, 255), 1, CV_AA);
	}

	vector<Vec2f> lines2;
	HoughLines(rSobel, lines2, laneAssist.rho2 + 1, CV_PI / laneAssist.theta2, laneAssist.max_lenght2, 0, 0);
	for (size_t i = lines2.size() - 2; i < lines2.size(); i++)
	{
		float rho2 = lines2[i][0], theta2 = lines2[i][1];
		Point pt1, pt2, pt3, pt4;
		double a = cos(theta2), b = sin(theta2);
		double x0 = a*rho2, y0 = b*rho2;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		pt3.x = pt1.x + 160;
		pt3.y = pt1.y;
		pt4.x = pt2.x + 160;
		pt4.y = pt2.y;
		line(lineSobel, pt3, pt4, Scalar(0, 255, 255), 1, CV_AA);
	}
	capture.setFrame(lineSobel);

	return capture;

}
