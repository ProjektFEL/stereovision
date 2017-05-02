
#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost;

class ProcessB : public IProcess {
private: 
	thread *t;
	property_tree::ptree pt;  // citac .ini suborov
	int ptX1, ptX2, ptY1, ptY2, gray, theta1, rho1, theta2, rho2, max_lenght1, max_lenght2, edmin, edmax;
	Mat copyLeft, copyRight;
	Mat laneAssistFrame;

		/*int ptX1 = 80, ptX2 = 80, ptY1 = 145, ptY2 = 145, gray = 15;
		 int theta1 = 56, rho1 = 0, theta2 = 19, rho2 = 0, max_lenght1 = 24, max_lenght2 = 27, edmin = 2, edmax = 255;*/
public:
	// konstruktor, nacitavanie atributov zo suboru
	ProcessB(){
		property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			ptX1 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.ptX1"));
			ptX2 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.ptX2"));
			ptY1 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.ptY1"));
			ptY2 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.ptY2"));
			gray = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.gray"));
			theta1 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.theta1"));
			theta2 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.theta2"));
			rho1 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.rho1"));
			rho2 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.rho2"));
			max_lenght1 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.max_lenght1"));
			max_lenght2 = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.max_lenght2"));
			edmin = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.edmin"));
			edmax = boost::lexical_cast<int>(pt.get<string>("LaneAssistKoef.edmax"));
		}
		catch (...)
		{
			cout << "Error in parsing LaneAssistKoef in ProcessB!" << endl;
		}
		cvNamedWindow("LaneDetect control", CV_WINDOW_AUTOSIZE);
		resizeWindow("LaneDetect control", 400, 500);
		moveWindow("LaneDetect control", 10, 0);
	}

	thread* run(mutex *z, Mat frameLeft, Mat frameRight){
		z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		z->unlock();
		t = new thread(&ProcessB::process, this, copyLeft, copyRight);
		return t;
	}

	void work(Mat frameLeft, Mat frameRight){
		for (int i = 1; i <= 100; i++)
		{
			cout << "ProcessB: " << i << endl;
		}
	}

	// working with frameLeft and process it to birdView and then lineDetect, frameRight is no-using
	void process(Mat frameLeft, Mat frameRight){
		//while (!GetAsyncKeyState(VK_ESCAPE))
			cvCreateTrackbar("gray", "LaneDetect control", &gray, 255);
			cvCreateTrackbar("ptX1", "LaneDetect control", &ptX1, 320);
			cvCreateTrackbar("ptY1", "LaneDetect control", &ptY1, 240);
			cvCreateTrackbar("ptX2", "LaneDetect control", &ptX2, 320);
			cvCreateTrackbar("ptY2", "LaneDetect control", &ptY2, 240);
			cvCreateTrackbar("theta1", "LaneDetect control", &theta1, 360);
			cvCreateTrackbar("rho1", "LaneDetect control", &rho1, 360);
			cvCreateTrackbar("line lenght1", "LaneDetect control", &max_lenght1, 500);
			cvCreateTrackbar("theta2", "LaneDetect control", &theta2, 500);
			cvCreateTrackbar("rho2", "LaneDetect control", &rho2, 500);
			cvCreateTrackbar("line lenght2", "LaneDetect control", &max_lenght2, 500);

			Mat birdViewCapture;
			cvtColor(frameLeft, frameLeft, COLOR_BGR2GRAY);
			//cvtColor(frameRight, frameRight, COLOR_BGR2GRAY);

			Point2f src[4], dst[4];
			src[0].x = (float)ptX1;
			src[0].y = (float)ptY1;
			src[1].x = (float)frameLeft.cols - ptX2;
			src[1].y = (float)ptY2;
			src[2].x = (float)frameLeft.cols;
			src[2].y = (float)frameLeft.rows;
			src[3].x = (float)0;
			src[3].y = (float)frameLeft.rows;

			dst[0].x = (float)0;
			dst[0].y = (float)0;
			dst[1].x = (float)320 - 1;
			dst[1].y = (float)0;
			dst[2].x = (float)320 - 1;
			dst[2].y = (float)240 - 1;
			dst[3].x = (float)0;
			dst[3].y = (float)240 - 1;

			Mat undistorted = Mat(cvSize(320, 240), CV_8UC1);
			warpPerspective(frameLeft, frameLeft, getPerspectiveTransform(src, dst), cvSize(320, 240));
			frameLeft.copyTo(birdViewCapture);
			// birdview vyssie

			// lanedetect nizsie
			inRange(birdViewCapture, 0, gray, birdViewCapture);
			GaussianBlur(birdViewCapture, birdViewCapture, Size(3, 3), 0, 0, BORDER_DEFAULT);
			Mat sobelx64f, abs_sobelx64f, sobel8u;
			Sobel(birdViewCapture, sobelx64f, birdViewCapture.depth(), 1, 0, 3);
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
			HoughLines(lSobel, lines1, rho1 + 1, CV_PI / theta1, max_lenght1, 0, 0);
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
			HoughLines(rSobel, lines2, rho2 + 1, CV_PI / theta2, max_lenght2, 0, 0);
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
			lineSobel.copyTo(laneAssistFrame);
	}

	 Mat getFrame(){
		 return laneAssistFrame;
	 }

	 Mat getObject() {
		 return laneAssistFrame;
	 }

	 ~ProcessB()
	 {}
};