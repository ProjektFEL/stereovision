#include "DisparityMap.h"

DisparityMap::DisparityMap()
{};

DisparityMap::DisparityMap(string path, DisparityAlghoritm disparityAlghoritm, Gui gui)
{
	this->rightCamera = VideoCapture(path);
	this->disparityAlghoritm = disparityAlghoritm;
	this->gui = gui;
};

DisparityMap::DisparityMap(string rigthPath, string leftPath, DisparityAlghoritm dispAlghoritm, Gui gui)
{
	this->rightCamera = VideoCapture(rigthPath);
	this->leftCamera = VideoCapture(leftPath);
	this->disparityAlghoritm = disparityAlghoritm;
	this->gui = gui;
};

DisparityMap::~DisparityMap()
{};

Mat DisparityMap::getLeftImage() const
{
	return leftImage;
};

Mat DisparityMap::getRightImage() const
{
	return rightImage;
};

void DisparityMap::setLeftImage(Mat leftImage)
{
	this->leftImage = leftImage;
};

void DisparityMap::setRightImage(Mat rightImage)
{
	this->rightImage = rightImage;
};

/*
bool DisparityMap::calculateOneCamera()
{
	while (1){
			rightCamera >> rightImage;
			rightCamera >> leftImage;
			rightCamera.read(rightImage);
			rightCamera.read(leftImage);

		cv::Rect roi1, roi2;
		double width = rightImage.cols;
		double height = rightImage.rows;
		Rect myROI(0, 0, width / 2, height);
		rightImage = rightImage(myROI);

		Rect myROI2((width / 2) - 1, 0, width / 2, height);
		leftImage = leftImage(myROI2);

		resize(leftImage, leftImage, Size(320, 240), 0, 0, INTER_CUBIC);
		resize(rightImage, rightImage, Size(320, 240), 0, 0, INTER_CUBIC);

		disparityMap = disparityAlghoritm.calculateSGBM(leftImage, rightImage, leftDisparity, rightDisparity);

		imshow("Lava kamera", leftImage);
		imshow("Prava kamera", rightImage);
		imshow("Disparitna mapa", disparityMap);
		char key = (char)waitKey(30);
		switch (key)
		{
		case 'q':
		case 'Q':
		case  27: //escape key
			return 0;
		}
	}
return 0;
};
*/

void DisparityMap::calculateOneCamera()
{
		rightCamera >> rightImage;
		rightCamera >> leftImage;
		rightCamera.read(rightImage);
		rightCamera.read(leftImage);

		cv::Rect roi1, roi2;
		int width = rightImage.cols;
		int height = rightImage.rows;
		Rect myROI(0, 0, width / 2, height);
		rightImage = rightImage(myROI);

		Rect myROI2((width / 2) - 1, 0, width / 2, height);
		leftImage = leftImage(myROI2);

		resize(leftImage, leftImage, Size(320, 240), 0, 0, INTER_CUBIC);
		resize(rightImage, rightImage, Size(320, 240), 0, 0, INTER_CUBIC);

		disparityMap = disparityAlghoritm.calculateSGBM(leftImage, rightImage, leftDisparity, rightDisparity);

};


void DisparityMap::calculateTwoCameras()
{

		rightCamera >> rightImage;
		rightCamera >> leftImage;
		disparityMap = disparityAlghoritm.calculateSGBM(leftImage, rightImage, leftDisparity, rightDisparity);

};

int DisparityMap::calculateDepthOneCamera()
{
	while (1){
		calculateOneCamera();
		bitwise_not(disparityMap, depthMap);
		applyColorMap(disparityMap, depthMap, COLORMAP_JET);
	
		gui.showDisparitySettings();

		imshow("Lava kamera", leftImage);
		imshow("Prava kamera", rightImage);
		imshow("Disparitna mapa", disparityMap);

		char key = (char)waitKey(30);
		switch (key)
		{
		case 'q':
		case 'Q':
		case  27:
			return 0;
		}
	}
	return 1;
};

int DisparityMap::calculateDepthTwoCameras()
{
		calculateTwoCameras();
		bitwise_not(disparityMap, depthMap);
		applyColorMap(disparityMap, depthMap, COLORMAP_JET);
};