#include "Calibration.h"
#include "structures.h"

using namespace cv;
using namespace std;

Calibration::Calibration()
{
}

Calibration::Calibration(VideoCapture capture)
{
	capture >> frame;
}
//getter
Mat Calibration::getFrame()
{
	return frame;
}
//setter
void Calibration::setFrame(Mat setFrame)
{
	frame = setFrame;
}


Calibration::~Calibration()
{
}

//funkcia pre disparitnu mapu
//rotacia vstupnych obrazkov podla koeficientov zo struktury, vracia kalibrovany obrázok
Calibration Calibration::calibFrame(Calibration capture, Mat M, Mat D, Mat R, Mat P)
{
	Mat frame = capture.getFrame();
	Size imageSize = frame.size();

	Mat rmap[1][1];


	Mat img1r;
	Mat map11, map12;

	initUndistortRectifyMap(M, D, R, P, imageSize, CV_16SC2, map11, map12);
	remap(frame, img1r, map11, map12, INTER_CUBIC);
	normalize(img1r, img1r, 0, 255, CV_MINMAX, CV_8U);
	//normalize(vzor, vzor,   0, 255, CV_MINMAX, CV_8U);

	capture.setFrame(img1r);
	return capture;
}


//ma vplyv na lane assist

Calibration Calibration::birdViewTransform(Calibration capture, laneAssistKoef laneAssist)
{

	Mat leftImage = capture.getFrame();
	cvtColor(leftImage, leftImage, COLOR_BGR2GRAY);

	Point2f src[4], dst[4];
	src[0].x = laneAssist.ptX1;
	src[0].y = laneAssist.ptY1;
	src[1].x = leftImage.cols - laneAssist.ptX2;
	src[1].y = laneAssist.ptY2;
	src[2].x = leftImage.cols;
	src[2].y = leftImage.rows;
	src[3].x = 0;
	src[3].y = leftImage.rows;

	dst[0].x = 0;
	dst[0].y = 0;
	dst[1].x = 320 - 1;
	dst[1].y = 0;
	dst[2].x = 320 - 1;
	dst[2].y = 240 - 1;
	dst[3].x = 0;
	dst[3].y = 240 - 1;

	Mat undistorted = Mat(cvSize(320, 240), CV_8UC1);
	warpPerspective(leftImage, leftImage, getPerspectiveTransform(src, dst), cvSize(320, 240));

	capture.setFrame(leftImage);

	return capture;
}
