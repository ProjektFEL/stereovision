

#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include <highgui.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace boost;
using namespace std;


class ProcessC : public IProcess {
private:
	thread* t;
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


public:
	// konstruktor, nacitavanie atributov zo suboru (este treba dorobit)
	ProcessC() {
		cny_threshold = 100;
		cny_thresholdL = 50;
		cny_thresholdH = 150;
		cny_apertureSize = 3;
		trsh_maxValue = 255;
		trsh_blockSize = 5;
		trsh_constant = 7;
		trsh_constant_Compt = 50;
		hough_rho = 1;
		hough_theta = CV_PI / 180;
		hough_tresh = 20;
		hough_minLength = 20;
		hough_maxLine = 300;
		angleMaxLeft = 0, angleMinLeft = 0, angleMaxRight = 0, angleMinRight = 0;
		low_r = 0, low_g = 170, low_b = 0;
		roiX = 50, roiY = 50, roiW = 100, roiH = 100;

		//namedWindow("LaneDetect control C", cv::WINDOW_AUTOSIZE);
		//cv::resizeWindow("LaneDetect control C", 400, 700);
		//cv::moveWindow("LaneDetect control C", 1300, 0);
		kernelSize = 7;
		Maska = 1;
		topLeftX = 95, topLeftY = 60, topRightX = 290, topRightY = 60;
	}

	thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight) {
		 z->try_lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		 z->unlock();
		t = new thread(&ProcessC::process, this, copyLeft, copyRight);
		return t;
	}

	void work(cv::Mat frameLeft, cv::Mat frameRight) {
		for (int i = 1; i <= 100; i++) {
			cout << "ProcessC: " << i << endl;
		}
	}

	void process(cv::Mat frameLeft, cv::Mat frameRight) {
		if(!frameLeft.empty() && !frameRight.empty()){
		// int low_r = 0, low_g = 200, low_b = 0;
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

		medianRightPoints = ascendPoints(medianRightPoints);
		medianLeftPoints = ascendPoints(medianLeftPoints);

		extendLeftPoints = extendPoints(imageRoi, medianLeftPoints);
		extendRightPoints = extendPoints(imageRoi, medianRightPoints);

		if (!extendRightPoints.empty() && !extendRightPoints.empty()) {
			drawMiddleLine(imageRoi, cv::Scalar(0, 0, 0));

			line(imageRoi, extendLeftPoints[0], extendLeftPoints[1], leftLineColor, 2,
				8, 0);
			line(imageRoi, extendRightPoints[0], extendRightPoints[1], rightLineColor,
				2, 8, 0);
			/*cout << "Pravy1"<< extendRightPoints[0] << endl;
			cout << "Pravy2" << extendRightPoints[1] << endl;*/

			result =
				calculateWheel(imageRoi, medianLeftPoints, leftLineColor,
				medianRightPoints, rightLineColor, yBase, yVertex);
		}
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
	vector<double> calculateWheel(cv::Mat image, vector<cv::Point> leftPoints,
		cv::Scalar leftLineColor, vector<cv::Point> rightPoints,
		cv::Scalar rightLineColor, int yBase, int yVertex) {
		vector<double> result;
		double coefficient, direction, alfa, delta;
		int xBS, xVS, yBS = yBase, yVS = yVertex, x0;

		cv::Point pointVL = findPointOnImage(image, yVS, leftLineColor);
		cv::Point pointVR = findPointOnImage(image, yVS, rightLineColor);
		cv::Point pointBL = findPointOnImage(image, yBS, leftLineColor);
		cv::Point pointBR = findPointOnImage(image, yBS, rightLineColor);

		xBS = (pointBL.x + pointBR.x) / 2;
		xVS = (pointVL.x + pointVR.x) / 2;

		coefficient = (double)(leftPoints[0].x + leftPoints[1].x) /
			(leftPoints[0].y + leftPoints[1].y);

		direction = (image.size().width / 2) > xBS ? 1 : 0;
		alfa = (double)(atan((double)(xVS - xBS) / (yBS - yVS)) * 180 / PI);
		delta = image.size().width / 2 - xBS;

		if (alfa > 0) { drawWheelAngle(image, alfa, xBS, yBS, xVS, yVS, cv::Scalar(0, 0, 255)); }
		else { drawWheelAngle(image, alfa, xBS, yBS, xVS, yVS, cv::Scalar(255, 0, 0)); }


		result = { alfa, delta, direction };

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

	~ProcessC() {}


};

