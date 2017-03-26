
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
	property_tree::ptree pt;  // citac .ini suborov
	int ptX1, ptX2, ptY1, ptY2, gray, theta1, rho1, theta2, rho2, max_lenght1, max_lenght2, edmin, edmax;
	Mat lineAssistFrame;

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
	}

	void process(Mat frameLeft,Mat frameRight){

		inRange(frameLeft, 0, gray, frameLeft);
		GaussianBlur(frameLeft, frameLeft, Size(3, 3), 0, 0, BORDER_DEFAULT);
		Mat sobelx64f, abs_sobelx64f, sobel8u;
		Sobel(frameLeft, sobelx64f, frameLeft.depth(), 1, 0, 3);
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
		lineSobel.copyTo(lineAssistFrame);
	 }

	 Mat getFrame(){
		 return lineAssistFrame;
	 }

	// este prerobit tuto funkciu, zla navratova hodnota
	 Mat getObject() {
		 return lineAssistFrame;
	 }

	 ~ProcessB()
	 {}
};