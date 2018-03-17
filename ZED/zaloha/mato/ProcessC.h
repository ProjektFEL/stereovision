

#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <stack>
#include <algorithm>
#include <math.h>

#define PI 3.14159265

using namespace boost;
using namespace std;


class ProcessC : public IProcess {
private:
	thread* t;
	property_tree::ptree pt;  // citac .ini suborov
	cv::Mat copyLeft, copyRight;
	cv::Mat imageROI, frame1, original, frameEroded, temp, img;
	int cny_threshold, cny_thresholdL, cny_thresholdH, cny_apertureSize,
		trsh_maxValue, trsh_blockSize, trsh_constant, trsh_constant_Compt;
	double hough_rho, hough_theta, hough_tresh, hough_minLength, hough_maxLine;
	int angleMaxLeft, angleMinLeft, angleMaxRight, angleMinRight;
	int angleMaxLeft_Compt, angleMinLeft_Compt, angleMaxRight_Compt,
		angleMinRight_Compt;
	int kernelSize, low_r, low_g, low_b;
	int roiX, roiY, roiW, roiH, Maska, topLeftX, topLeftY, topRightX, topRightY;
	//stack<vector<cv::Point>> *bufferExtendRightPoints, *bufferExtendLeftPoints;
	stack<cv::Point> *bufferPointsVL, *bufferPointsVR, *bufferPointsBL, *bufferPointsBR;
	stack <double> bufferAngles;

public:
	// konstruktor, nacitavanie atributov zo suboru (este treba dorobit)
	ProcessC() {
		//bufferExtendRightPoints = new stack<vector<cv::Point>>();
		//bufferExtendLeftPoints = new stack<vector<cv::Point>>();
		bufferPointsVL = new stack<cv::Point>();
		bufferPointsVR = new stack<cv::Point>();
		bufferPointsBL = new stack<cv::Point>();
		bufferPointsBR = new stack<cv::Point>();


		property_tree::ini_parser::read_ini("config.ini", pt);  // nacitavanie zo suboru config.ini
		try
		{   //nie som si isty ci potrebujeme mat vsetky atributy nacitane
			cny_threshold = boost::lexical_cast<int>(pt.get<string>("ProcessC.cny_threshold"));
			cny_thresholdL = boost::lexical_cast<int>(pt.get<string>("ProcessC.cny_thresholdL"));
			cny_thresholdH = boost::lexical_cast<int>(pt.get<string>("ProcessC.cny_thresholdH"));
			cny_apertureSize = boost::lexical_cast<int>(pt.get<string>("ProcessC.cny_apertureSize"));
			trsh_maxValue = boost::lexical_cast<int>(pt.get<string>("ProcessC.trsh_maxValue"));
			trsh_blockSize = boost::lexical_cast<int>(pt.get<string>("ProcessC.trsh_blockSize"));
			trsh_constant = boost::lexical_cast<int>(pt.get<string>("ProcessC.trsh_constant"));
			trsh_constant_Compt = boost::lexical_cast<int>(pt.get<string>("ProcessC.trsh_constant_Compt"));
			hough_rho = boost::lexical_cast<double>(pt.get<string>("ProcessC.hough_rho"));
			hough_theta = CV_PI / 180;
			hough_tresh = boost::lexical_cast<double>(pt.get<string>("ProcessC.hough_tresh"));
			hough_minLength = boost::lexical_cast<double>(pt.get<string>("ProcessC.hough_minLength"));
			hough_maxLine = boost::lexical_cast<double>(pt.get<string>("ProcessC.hough_maxLine"));
			angleMaxLeft = boost::lexical_cast<int>(pt.get<string>("ProcessC.angleMaxLeft"));
			angleMinLeft = boost::lexical_cast<int>(pt.get<string>("ProcessC.angleMinLeft"));
			angleMaxRight = boost::lexical_cast<int>(pt.get<string>("ProcessC.angleMaxRight"));
			angleMinRight = boost::lexical_cast<int>(pt.get<string>("ProcessC.angleMinRight"));
			low_r = boost::lexical_cast<int>(pt.get<string>("ProcessC.low_r"));
			low_g = boost::lexical_cast<int>(pt.get<string>("ProcessC.low_g"));
			low_b = boost::lexical_cast<int>(pt.get<string>("ProcessC.low_b"));
			roiX = boost::lexical_cast<int>(pt.get<string>("ProcessC.roiX"));
			roiY = boost::lexical_cast<int>(pt.get<string>("ProcessC.roiY"));
			roiW = boost::lexical_cast<int>(pt.get<string>("ProcessC.roiW"));
			roiH = boost::lexical_cast<int>(pt.get<string>("ProcessC.roiH"));
			kernelSize = boost::lexical_cast<int>(pt.get<string>("ProcessC.kernelSize"));
			Maska = boost::lexical_cast<int>(pt.get<string>("ProcessC.Maska"));
			topLeftX = boost::lexical_cast<int>(pt.get<string>("ProcessC.topLeftX"));
			topLeftY = boost::lexical_cast<int>(pt.get<string>("ProcessC.topLeftY"));
			topRightX = boost::lexical_cast<int>(pt.get<string>("ProcessC.topRightX"));
			topRightY = boost::lexical_cast<int>(pt.get<string>("ProcessC.topRightY"));
		}
		catch (...)
		{
			cout << "Error in parsing LaneAssistKoef in ProcessB!" << endl;
		}
		//cvNamedWindow("LaneDetect control C", CV_WINDOW_AUTOSIZE);
		cv::resizeWindow("LaneDetect control C", 400, 700);
		cv::moveWindow("LaneDetect control C", 1300, 0);
	}

	thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight) {
		// z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		// z->unlock();
		t = new thread(&ProcessC::process, this, copyLeft, copyRight);
		return t;
	}

	void work(cv::Mat frameLeft, cv::Mat frameRight) {
		for (int i = 1; i <= 100; i++) {
			cout << "ProcessC: " << i << endl;
		}
	}

	void process(cv::Mat frameLeft, cv::Mat frameRight) {
		int high_r = 255, high_g = 255, high_b = 255;
		cv::Mat frameHls, frametresh, frameGauss, frameAdded, frameCanny, frameRoi;

		cvCreateTrackbar("roiX", "LaneDetect control C", &roiX, frameLeft.cols);
		cvCreateTrackbar("roiY", "LaneDetect control C", &roiY, frameLeft.rows);

		roiW = frameLeft.cols - roiX;
		roiH = frameLeft.rows - roiY;
		// cvCreateTrackbar("roiW", "LaneDetect control C", &roiW, frameLeft.cols);
		// cvCreateTrackbar("roiH", "LaneDetect control C", &roiH, frameLeft.rows);
		cvCreateTrackbar("low_r", "LaneDetect control C", &low_r, 255);
		cvCreateTrackbar("low_g", "LaneDetect control C", &low_g, 255);
		cvCreateTrackbar("low_b", "LaneDetect control C", &low_b, 255);
		cvCreateTrackbar("kernelSize", "LaneDetect control C", &kernelSize, 21);
		cvCreateTrackbar("cny_Lthreshold", "LaneDetect control C", &cny_thresholdL,
			200);
		cvCreateTrackbar("cny_Hthreshold", "LaneDetect control C", &cny_thresholdH,
			200);

		cv::Rect region_of_interest = cv::Rect(roiX, roiY, roiW, roiH);
		cv::Mat imageRoi = frameLeft(region_of_interest);

		if (kernelSize % 2 == 0) kernelSize++;
		cvtColor(imageRoi, frameHls, cv::COLOR_RGB2HLS);

		inRange(frameHls, cv::Scalar(low_b, low_g, low_r),
			cv::Scalar(high_b, high_g, high_r), frametresh);
		cv::imshow("frametresh", frametresh);
		bitwise_and(frameHls, frameHls, frameAdded, frametresh);
		cvtColor(frameAdded, frameAdded, cv::COLOR_RGB2GRAY);
		GaussianBlur(frameAdded, frameGauss, cv::Size(kernelSize, kernelSize), 0, 0);
		Canny(frameGauss, frameCanny, cny_thresholdL, cny_thresholdH,
			cny_apertureSize);


		cvCreateTrackbar("topLeftX", "LaneDetect control C", &topLeftX,
			frameCanny.cols);
		cvCreateTrackbar("topLeftY", "LaneDetect control C", &topLeftY,
			frameCanny.rows);
		cvCreateTrackbar("topRightX", "LaneDetect control C", &topRightX,
			frameCanny.cols);
		cvCreateTrackbar("topRightY", "LaneDetect control C", &topRightY,
			frameCanny.rows);
		cvCreateTrackbar("Maska", "LaneDetect control C", &Maska, 1);

		if (Maska == 1) {
			cv::Mat mask = computeMask(frameCanny);
			frameCanny.copyTo(frameRoi, mask);

		}
		else if (Maska == 0) {
			frameCanny.copyTo(frameRoi);
		}

		if (!frameRoi.empty()) {
			laneAssist(frameRoi, imageRoi);
			imageRoi.copyTo(frame1);
		}
	}

	cv::Mat computeMask(cv::Mat canny) {
		cv::Mat black(canny.rows, canny.cols, canny.type(), cv::Scalar::all(0));
		cv::Mat mask(canny.rows, canny.cols, CV_8UC1, cv::Scalar(0));

		cv::Point P1(canny.cols * 0, canny.rows);  // bottomleft
		cv::Point P2(topLeftX, topLeftY);          // top left
		cv::Point P3(topRightX, topRightY);        // top right
		cv::Point P4(canny.cols, canny.rows);      // bottom right

		vector<vector<cv::Point>> co_ordinates;
		co_ordinates.push_back(vector<cv::Point>());
		co_ordinates[0].push_back(P1);
		co_ordinates[0].push_back(P2);
		co_ordinates[0].push_back(P3);
		co_ordinates[0].push_back(P4);
		drawContours(mask, co_ordinates, 0, cv::Scalar(255), CV_FILLED, 8);

		return mask;

	}

	void laneAssist(cv::Mat frameRoi, cv::Mat imageRoi) {
		vector<vector<cv::Point>> leftPartPoints, rightPartPoints;
		vector<cv::Point> medianLeftPoints, medianRightPoints, extendLeftPoints,
			extendRightPoints;
		vector<double> result;

		cv::Scalar leftLineColor(255, 0, 0), rightLineColor(0, 0, 255);
		int yBase = (int)(imageRoi.size().height - 50), yVertex = 20;

		cv::Mat leftPartImg, rightPartImg;
		leftPartImg = cutImage(frameRoi, 0, 0, frameRoi.size().width / 2,
			frameRoi.size().height);
		rightPartImg = cutImage(frameRoi, frameRoi.size().width / 2, 0,
			frameRoi.size().width / 2, frameRoi.size().height);

		leftPartPoints = findLines(leftPartImg);
		medianLeftPoints = computeMedian(leftPartPoints);
		rightPartPoints = findLines(rightPartImg);
		medianRightPoints = computeMedian(rightPartPoints);

		medianRightPoints[0].x += frameRoi.size().width / 2;
		medianRightPoints[1].x += frameRoi.size().width / 2;

		////removing coordinates errors
		//removeCoordinatesErrors(medianRightPoints, *bufferExtendLeftPoints);
		//removeCoordinatesErrors(medianLeftPoints, *bufferExtendRightPoints);

		//vector<cv::Point> resultLeftPoints, resultRightPoints;

		//resultLeftPoints = bufferExtendLeftPoints->top();
		//resultRightPoints = bufferExtendRightPoints->top();


		medianRightPoints = ascendPoints(medianRightPoints);
		medianLeftPoints = ascendPoints(medianLeftPoints);

		extendLeftPoints = extendPoints(imageRoi, medianLeftPoints);
		extendRightPoints = extendPoints(imageRoi, medianRightPoints);

		drawMiddleLine(imageRoi, cv::Scalar(0, 0, 0));

		if (!extendRightPoints.empty()) {
			line(imageRoi, extendLeftPoints[0], extendLeftPoints[1], leftLineColor, 2,
				8, 0);
		}

		if (!extendLeftPoints.empty()) {
			line(imageRoi, extendRightPoints[0], extendRightPoints[1], rightLineColor,
				2, 8, 0);
		}
			//cout << "Pravy1"<< extendRightPoints[0] << endl;
			//cout << "Pravy2" << extendRightPoints[1] << endl;
			//cout << "lavy1" << extendRightPoints[0] << endl;
			//cout << "lavy2" << extendRightPoints[1] << endl;

			result =
				calculateWheelAngle(imageRoi, medianLeftPoints, leftLineColor,
				medianRightPoints, rightLineColor, yBase, yVertex);

				//cout << "uhol: " << bufferAngles.top() << endl;
		//}
	}

	// cut image
	cv::Mat cutImage(cv::Mat input, int x, int y, int width, int height) {
		return input(cv::Rect(x, y, width, height));
	}

	// join images vertically
	cv::Mat mergeImages(cv::Mat firstImg, cv::Mat secondImg) {
		cv::Size firstSize = firstImg.size();
		cv::Size secondSize = secondImg.size();
		cv::Mat mergeImg(firstSize.height, firstSize.width + secondSize.width, CV_8UC3);
		cv::Mat left(mergeImg, cv::Rect(0, 0, firstSize.width, firstSize.height));
		firstImg.copyTo(left);
		cv::Mat right(mergeImg,
			cv::Rect(firstSize.width, 0, secondSize.width, secondSize.height));
		secondImg.copyTo(right);
		return mergeImg;
	}

	// quasi - median
	vector<cv::Point> computeMedian(vector<vector<cv::Point>> points) {
		vector<cv::Point> medianPoints(2), middlePoints(2);

		middlePoints = computeMiddlePoints(points);
		points = quicksort(points, middlePoints);

		int medianNumber = (int)(points.size() / 2);

		if (points.size() > 0 && middlePoints[medianNumber] != cv::Point(0, 0)) {
			medianPoints = { points[medianNumber][0], points[medianNumber][1] };
		}

		return medianPoints;
	}

	// calculate middle of points
	vector<cv::Point> computeMiddlePoints(vector<vector<cv::Point>> points) {
		vector<cv::Point> middlePoints;
		middlePoints.resize(points.size());
		for (int i = 0; i < points.size(); i++) {
			if (points[i].at(0) != cv::Point(0, 0) && points[i].at(1) != cv::Point(0, 0)) {
				int middleX = (points[i][0].x + points[i][1].x) / 2;
				int middleY = (points[i][0].y + points[i][1].y) / 2;
				middlePoints[i] = cv::Point(middleX, middleY);
			}
		}
		return middlePoints;
	}

	void removeCoordinatesErrors(vector<cv::Point> extendPoints, stack<vector<cv::Point>>  &bufferExtendPoints)
	{
		vector<cv::Point> topBuffer(2), temp(2);
		cv::Point firstPoint, secondPoint;
		if (!extendPoints.empty())
		{
			firstPoint = extendPoints[0];
			secondPoint = extendPoints[1];

			if ((firstPoint.x > roiW * -4 && firstPoint.x < roiW * 4) && (firstPoint.y > roiH * -4 && firstPoint.y < roiH * 4))
			{
				temp[0] = firstPoint;
			}
			else
			{
				temp[0] = cv::Point(0, 0);
			}
			if ((secondPoint.x > roiW * -4 && secondPoint.x < roiW * 4) && (secondPoint.y > roiH * -4 && secondPoint.y < roiH * 4))
			{
				temp[1] = secondPoint;
			}
			else
			{
				temp[1] = cv::Point(0, 0);
			}

			if (!bufferExtendPoints.empty())
			{
				topBuffer = bufferExtendPoints.top();
				if (temp[0].x == 0 && temp[0].y == 0 && (topBuffer[0].x != temp[0].x || topBuffer[0].y != temp[0].y))
				{
					temp[0] = topBuffer[0];
				}
				if (temp[1].x == 0 && temp[1].y == 0 && (topBuffer[1].x != temp[1].x || topBuffer[1].y != temp[1].y))
				{
					temp[1] = topBuffer[1];
				}
			}
			bufferExtendPoints.push(temp);
		}
		if (bufferExtendPoints.size() > 3)
		{
			temp = bufferExtendPoints.top();
			bufferExtendPoints = stack<vector<cv::Point>>();
			bufferExtendPoints.push(temp);

		}
	}

	// calculate quicksort ascedent according to middle points
	vector<vector<cv::Point>> quicksort(vector<vector<cv::Point>> points,
		vector<cv::Point> middlePoints) {
		cv::Point tempX, tempY;

		for (int i = 1; i < points.size(); i++) {
			for (int j = i; j > 0; j--) {
				if (middlePoints[i].x != 0 && middlePoints[i].y != 0) {
					if (middlePoints[j].y < middlePoints[j - 1].y) {
						// start point
						tempX = points[j][0];
						points[j][0] = points[j - 1][0];
						points[j - 1][0] = tempX;
						// end point
						tempY = points[j][1];
						points[j][1] = points[j - 1][1];
						points[j - 1][1] = tempY;
					}
				}
			}
		}
		return points;
	}

	/* chyba popis
	*/
	vector<double> calculateWheelAngle(cv::Mat image, vector<cv::Point> leftPoints,
		cv::Scalar leftLineColor, vector<cv::Point> rightPoints,
		cv::Scalar rightLineColor, int yBase, int yVertex) {
		int yBS = yBase, yVS = yVertex;

		cv::Point pointVL = findPointOnImage(image, yVS, leftLineColor);
		cv::Point pointVR = findPointOnImage(image, yVS, rightLineColor);
		cv::Point pointBL = findPointOnImage(image, yBS, leftLineColor);
		cv::Point pointBR = findPointOnImage(image, yBS, rightLineColor);

		//removing findingpoints errors
		removeFindedPointErrors(*bufferPointsVL, pointVL);
		removeFindedPointErrors(*bufferPointsVR, pointVR);
		removeFindedPointErrors(*bufferPointsBL, pointBL);
		removeFindedPointErrors(*bufferPointsBR, pointBR);
		pointVL = bufferPointsVL->top();
		pointVR = bufferPointsVR->top();
		pointBL = bufferPointsBL->top();
		pointBR = bufferPointsBR->top();


		int xBS = (pointBL.x + pointBR.x) / 2;
		int xVS = (pointVL.x + pointVR.x) / 2;

		double coefficient = (double)(leftPoints[0].x + leftPoints[1].x) /
			(leftPoints[0].y + leftPoints[1].y);

		//direction = (image.size().width / 2) > xBS ? 1 : 0;
		double angle = (double)(atan((double)(xVS - xBS) / (yBS - yVS)) * 180 / PI);
		double delta = image.size().width / 2 - xBS;

		//removing angle errors
		removeWheelAnglesErrors(angle);

		//show result
		if (bufferAngles.top() > 0) { drawWheelAngle(image, bufferAngles.top(), xBS, yBS, xVS, yVS, rightLineColor); }
		else { drawWheelAngle(image, bufferAngles.top(), xBS, yBS, xVS, yVS, leftLineColor); }

		//result = { angle, delta, direction };
		vector<double> result = { angle, delta };

		return result;
	}

	void drawWheelAngle(cv::Mat image, double alfa, int xBS, int yBS, int xVS,
		int yVS, cv::Scalar color) {
		vector<cv::Point> middlePointsBS = { cv::Point(xBS, 0), cv::Point(xBS, yBS) };
		vector<cv::Point> PointsS = { cv::Point(xBS, yBS), cv::Point(xVS, yVS) };
		vector<cv::Point> extendPointsS = extendPoints(image, PointsS);

		extendPointsS = ascendPoints(extendPointsS);
		extendPointsS.at(1) = middlePointsBS.at(1);

		line(image, middlePointsBS.at(0), middlePointsBS.at(1), color, 2, 8, 0);
		line(image, extendPointsS[0], extendPointsS[1], color, 2, 8, 0);
		ellipse(image, cv::Point(xBS, yBS), cv::Size(150, 150), alfa, -90, -90 - alfa,
			color, 3, 8);
	}

	void drawMiddleLine(cv::Mat image, cv::Scalar color) {
		cv::Point middlePointB(image.size().width / 2, image.size().height);
		cv::Point middlePointV(image.size().width / 2, 0);
		cv::line(image, middlePointB, middlePointV, cv::Scalar(255, 255, 255), 2, 8, 0);
	}

	vector<vector<cv::Point>> findLines(cv::Mat image) {
		vector<cv::Vec4i> hughLines;
		vector<vector<cv::Point>> points;

		HoughLinesP(image, hughLines, hough_rho, hough_theta, hough_tresh,
			hough_minLength, hough_maxLine);
		points.resize(hughLines.size(), vector<cv::Point>(2));

		for (size_t i = 0; i < hughLines.size(); i++) {
			cv::Vec4i inputPoints = hughLines[i];
			if (inputPoints != cv::Vec4i(0)) {
				cv::Point startPoint = cv::Point(inputPoints[0], inputPoints[1]);
				cv::Point endPoint = cv::Point(inputPoints[2], inputPoints[3]);
				points.at(i) = { startPoint, endPoint };
			}
		}
		return points;
	}


	//metoda ukada do zasobnika nenulove body
	void removeFindedPointErrors(stack<cv::Point> &bufferPoints, cv::Point findedPoint)
	{
		cv::Point temp;
		if (bufferPoints.empty())
		{
			//cout << "ukladam do buffera: " << findedPoint << endl;
			bufferPoints.push(findedPoint);
		}
		else
		{
			//cout << "vrchol zasobnika: " << bufferPoints.top() << endl;
			if (!findedPoint.x == 0 && !findedPoint.y == 0)
			{
				//cout << "-----------------" << endl;
				//cout << "ukladam do buffera: " << findedPoint << endl;
				bufferPoints.push(findedPoint);
			}
		}

		if (bufferPoints.size() > 3)
		{
			temp = bufferPoints.top();
			bufferPoints = stack<cv::Point>();
			bufferPoints.push(temp);
			//cout << bufferPoints.size() << endl;
		}
	}

	/*
	metoda kontroluje posledny ulozeny uhol v bufferi a porovnava ho s aktualnym v pripade ze je aktualny <> aspon o 5 stupnov tak
	ho ulozi na vrchol zasobnika a okrem toho ho vrati
	*/
	void removeWheelAnglesErrors(double angle)
	{
		double pom;
		if (bufferAngles.empty() == false)
		{
			if (angle != bufferAngles.top())
				{
				//compare actual angle and angle on a top of buffer if is bigger than 5 degrees
				if (angle > bufferAngles.top() && angle - bufferAngles.top() > 5)
				{
					bufferAngles.push(angle);
				}
				//compare actual angle and angle on a top of buffer if is smaller than 5 degrees
				else if (angle < bufferAngles.top() && angle - bufferAngles.top()< 5)
				{
					bufferAngles.push(angle);
				}
			}
		}
		else
		{
			bufferAngles.push(angle);
		}

		if (angle  < 5 && angle > -5)
		{
			bufferAngles.push(0);
		}

		if (bufferAngles.size() > 3)
		{
			pom = bufferAngles.top();
			bufferAngles = stack<double>();
			bufferAngles.size();
			bufferAngles.push(pom);
		}
	}


	/*! \fn vector<Point> ascendPoints(vector<Point> points)
	\brief Sort points ascending to vector according to y values.
	\param points Vector of two points with x and y coordinates.
	*/
	vector<cv::Point> ascendPoints(vector<cv::Point> points) {
		cv::Point temp;
		if (points[0].y > points[1].y) {
			temp = points[0];
			points[0] = points[1];
			points[1] = temp;
		}
		return points;
	}

	/*! \fn vector<Point> extendsLines(Mat imageRoi, vector<Point> median)
	\brief Extends lines on image according to input points.
	\param image Input image.
	\param points Vector of two points with x and y coordinates.
	*/
	vector<cv::Point> extendPoints(cv::Mat image, vector<cv::Point> points) {
		vector<cv::Point> extendPoints(2);

		double slope = (double)(points.at(1).y - points.at(0).y) /
			(points.at(1).x - points.at(0).x);

		cv::Point startPoint(0, 0), endPoint = cv::Point(image.cols, image.rows);
		startPoint.y = -(points.at(0).x - startPoint.x) * slope + points.at(0).y;
		endPoint.y = -(points.at(1).x - endPoint.x) * slope + points.at(1).y;
		extendPoints = { startPoint, endPoint };
		return extendPoints;
	}

	/*! \fn Point findPointOnImage(Mat image, int row, Scalar color)
	\brief Find point on image according to specified row and input color.
	\param image Input image.
	\param row Number of a row on a image.
	\param color Color, which he find in a row.
	*/
	cv::Point findPointOnImage(cv::Mat image, int row, cv::Scalar targetColor) {
		cv::Point point;
		for (int j = 0; j < image.cols; j++) {
			cv::Vec3b pointColor = image.at<cv::Vec3b>(cv::Point(j, row));
			if (pointColor.val[0] == targetColor[0] &&
				pointColor.val[1] == targetColor[1] &&
				pointColor.val[2] == targetColor[2]) {
				point = cv::Point(j, row);
				return point;
			}
		}
		return point;
	}

	cv::Mat getFrame() { return frame1; }

	~ProcessC() {
		//delete bufferExtendRightPoints;
		//bufferExtendRightPoints = NULL;
		//delete bufferExtendLeftPoints;
		//bufferExtendLeftPoints = NULL;
		delete bufferPointsVL;
		bufferPointsVL = NULL;
		delete bufferPointsVR;
		bufferPointsVR = NULL;
		delete bufferPointsBL;
		bufferPointsBL = NULL;
		delete bufferPointsBR;
		bufferPointsBR = NULL;

	}
	double getAngle(){
	return bufferAngles.top();
	}
	cv::Mat getObject() {
			 return frame1;
	 }


};

