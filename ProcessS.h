
#include "IProcess.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

using namespace std;

class ProcessS : public IProcess {
private:

	cv::Mat copyLeft, copyRight, copyDisparity, sobel, sobely;
	cv::Mat v_disparity, u_disparity;
	cv::Mat thresBlizke, thresStredne, gray, thresBlizkeMaska, thresBlizkeRange;
	cv::Mat leftBlizke,middleBlizke,rightBlizke, leftStredne, middleStredne,rightStredne;
	int percentoBlizke, percentoStredne;
	string napisBlizke, napisStredne;
	cv::Mat  ObjectCoordinates;
	int aB,bB,cB,aS,bS,cS;

public:
	ProcessS(){

percentoBlizke = 10;
percentoStredne = 10;

	}

	~ProcessS()
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
/*
int ddepth = CV_16S;
disparity.convertTo(disparity, CV_16S);
cv::Mat abs_grad_x, abs_grad_y;
cv::Mat grad_x, grad_y;
	int scale = 1;
	int delta = 5;
cv::Sobel(disparity,sobel,ddepth,0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
//cv::convertScaleAbs( grad_x, abs_grad_x );
sobel.convertTo(sobel, CV_8UC1);*/
cv::Rect rectLeft, rectMiddle, rectRight;
int rows, cols;
rows = disparity.rows;
cols = disparity.cols;
int tretina = (int)round(cols/3);

/*vytvorenie Rect na tretiny*/
rectLeft = cv::Rect(0,0,tretina,rows);
rectMiddle = cv::Rect(tretina,0,tretina,rows);
rectRight = cv::Rect((tretina*2),0,tretina,rows);

cv::cvtColor(disparity,gray, cv::COLOR_BGR2GRAY);
gray.convertTo(gray, CV_8UC1);
//cv::Sobel(sobel,sobely,CV_64F,1,1,5,1,0,cv::BORDER_DEFAULT);
//cv::imshow("sobel",sobely);
//sobely.convertTo(sobely, CV_64F);
/*cv::threshold(gray,thresStredne,175,214,cv::THRESH_BINARY);
cv::threshold(gray,thresBlizke,215,255,cv::THRESH_BINARY);*/
cv::inRange(gray,cv::Scalar(185),cv::Scalar(225),thresStredne);
cv::inRange(gray,cv::Scalar(225),cv::Scalar(255),thresBlizke);

/* som skusal pomocou sobel odstranit gradient
thresBlizkeMaska.release();
gray.copyTo(thresBlizkeMaska,thresBlizkeRange);
cv::imshow("thresBlizkeMaska",thresBlizkeMaska);
thresBlizkeMaska.convertTo(thresBlizkeMaska, CV_64F);
cv::Sobel(thresBlizkeMaska,thresBlizke,CV_64F,1,0,7,1,0,cv::BORDER_DEFAULT);
//sobely.convertTo(sobely, CV_8UC1);
cv::imshow("sobely",thresBlizke);
*/

/*rozdelenie obrazu na 3 Mat pre blizke*/
leftBlizke = thresBlizke(rectLeft);
middleBlizke = thresBlizke(rectMiddle);
rightBlizke = thresBlizke(rectRight);
/*rozdelenie obrazu na 3 Mat pre blizke*/
leftStredne = thresStredne(rectLeft);
middleStredne = thresStredne(rectMiddle);
rightStredne = thresStredne(rectRight);
/*spocitanie vsetkych bielich bodov pre Blizke*/
int countWhiteLeftBlizke, countWhiteMiddleBlizke, countWhiteRightBlizke;
countWhiteLeftBlizke=cv::countNonZero(leftBlizke);
countWhiteMiddleBlizke=cv::countNonZero(middleBlizke);
countWhiteRightBlizke=cv::countNonZero(rightBlizke);
/*spocitanie vsetkych bielich bodov pre Stredne*/
int countWhiteLeftStredne, countWhiteMiddleStredne, countWhiteRightStredne;
countWhiteLeftStredne=cv::countNonZero(leftStredne);
countWhiteMiddleStredne=cv::countNonZero(middleStredne);
countWhiteRightStredne=cv::countNonZero(rightStredne);
/*spocitanie vsetkych bodov pre Blizke*/
int countAllLeftBlizke, countAllMiddleBlizke, countAllRightBlizke;
countAllLeftBlizke = leftBlizke.cols * leftBlizke.rows;
countAllMiddleBlizke = middleBlizke.cols * middleBlizke.rows;
countAllRightBlizke = rightBlizke.cols * rightBlizke.rows;
/*spocitanie vsetkych bodov pre Stredne*/
int countAllLeftStredne, countAllMiddleStredne, countAllRightStredne;
countAllLeftStredne = leftStredne.cols * leftStredne.rows;
countAllMiddleStredne = middleStredne.cols * middleStredne.rows;
countAllRightStredne = rightStredne.cols * rightStredne.rows;
/*pocitanie percentualne kolko bielych bodov zabera z celej casti obrazu*/
double hodnotaLeftBlizke, hodnotaMiddleBlizke, hodnotaRightBlizke, hodnotaLeftStredne, hodnotaMiddleStredne, hodnotaRightStredne;
cv::namedWindow("thres", cv::WINDOW_AUTOSIZE);
cvCreateTrackbar("Percento blizke", "thres", &percentoBlizke, 100);
cvCreateTrackbar("Percento Stredne", "thres", &percentoStredne, 100);
/*percento blizkych objektov*/
hodnotaLeftBlizke = (countAllLeftBlizke/100)*percentoBlizke;
hodnotaMiddleBlizke = (countAllMiddleBlizke/100)*percentoBlizke;
hodnotaRightBlizke = (countAllRightBlizke/100)*percentoBlizke;
/*percento strednych objektov*/
hodnotaLeftStredne = (countAllLeftStredne/100)*percentoStredne;
hodnotaMiddleStredne = (countAllMiddleStredne/100)*percentoStredne;
hodnotaRightStredne = (countAllRightStredne/100)*percentoStredne;
aB=0;
bB=0;
cB=0;
aS=0;
bS=0;
cS=0;
/*vypisi na obraz disparity*/
napisBlizke = "Obstacle ";
if(countWhiteLeftBlizke < hodnotaLeftBlizke && countWhiteMiddleBlizke < hodnotaMiddleBlizke && countWhiteRightBlizke < hodnotaRightBlizke)
{
napisBlizke = "None";
}else
if (countWhiteLeftBlizke > hodnotaLeftBlizke)
{
napisBlizke += "Left ";
aB = 1;
}
if(countWhiteMiddleBlizke > hodnotaMiddleBlizke)
{
napisBlizke += "Middle ";
bB=1;
}
if(countWhiteRightBlizke > hodnotaRightBlizke)
{
napisBlizke += "Right ";
cB=1;
}

/*vypisi na obraz disparity*/
napisStredne = "Obstacle ";
if(countWhiteLeftStredne < hodnotaLeftStredne && countWhiteMiddleStredne < hodnotaMiddleStredne && countWhiteRightStredne < hodnotaRightStredne)
{
napisStredne = "None";
}else
if (countWhiteLeftStredne > hodnotaLeftStredne)
{
napisStredne += "Left ";
aS = 1;
}
if(countWhiteMiddleStredne > hodnotaMiddleStredne)
{
napisStredne += "Middle ";
bS=1;
}
if(countWhiteRightStredne > hodnotaRightStredne)
{
napisStredne += "Right ";
cS=1;
}

cv::putText(thresBlizke,napisBlizke,cvPoint(0,25),cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(255,255,255), 2);
cv::putText(thresStredne,napisStredne,cvPoint(0,25),cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(255,255,255), 2);

cv::line(thresBlizke, cv::Point(tretina,0), cv::Point(tretina,leftBlizke.rows), cv::Scalar(255,255,255),1,CV_AA,0);
cv::line(thresBlizke, cv::Point(tretina*2,0), cv::Point(tretina*2,leftBlizke.rows), cv::Scalar(255,255,255),1,CV_AA,0);
cv::line(thresStredne, cv::Point(tretina,0), cv::Point(tretina,leftStredne.rows), cv::Scalar(255,255,255),1,CV_AA,0);
cv::line(thresStredne, cv::Point(tretina*2,0), cv::Point(tretina*2,leftStredne.rows), cv::Scalar(255,255,255),1,CV_AA,0);
ObjectCoordinates = cv::Mat_<int>(2, 3);
ObjectCoordinates.at<int>(0,0) = aS;
ObjectCoordinates.at<int>(0,1) = bS;
ObjectCoordinates.at<int>(0,2) = cS;
ObjectCoordinates.at<int>(1,0) = aB;
ObjectCoordinates.at<int>(1,1) = bB;
ObjectCoordinates.at<int>(1,2) = cB;

//cout << ObjectCoordinates << endl;

	 }

	cv::Mat getFrame(){
		 return thresBlizke;
	 }

	 // este prerobit tuto funkciu, zla navratova hodnota
	cv::Mat getObject() {
			 return ObjectCoordinates;
	 }
	 double getAngle(){
	return 0;
	}

	cv::Mat getDisparity(){
		 return thresStredne;
	 }

};
