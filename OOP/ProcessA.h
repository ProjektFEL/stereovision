
#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost;

class ProcessA : public IProcess {
private:
	property_tree::ptree pt;  // citac .ini suborov
	int ptX1, ptX2, ptY1, ptY2, gray, theta1, rho1, theta2, rho2, max_lenght1, max_lenght2, edmin, edmax;
	Mat birdViewCapture;

	//int ptX1 = 80, ptX2 = 80, ptY1 = 145, ptY2 = 145, gray = 15;
	//	 int theta1 = 56, rho1 = 0, theta2 = 19, rho2 = 0, max_lenght1 = 24, max_lenght2 = 27, edmin = 2, edmax = 255;	
public:
	 
	// konstruktor, nacitavanie atributov zo suboru
	ProcessA(){
		property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{	//nie som si isty ci potrebujeme mat vsetky atributy nacitane
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
			cout << "Error in parsing LaneAssistKoef in ProcessA!" << endl;
		}
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

	 // este prerobit tuto funkciu, zla navratova hodnota
		 Mat getObject() {
		 return birdViewCapture;
	 }

		 ~ProcessA()
		 {}


};