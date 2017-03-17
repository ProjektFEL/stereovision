#include "DepthMap.h"
#include "structures.h"


using namespace cv;
using namespace std;

DepthMap::DepthMap()
{
}


DepthMap::~DepthMap()
{
}

DepthMap::DepthMap(VideoCapture capture)
{
	capture >> frame;
}
//getter
Mat DepthMap::getFrame()
{
	return frame;
}

//setter
void DepthMap::setFrame(Mat setFrame)
{
	frame = setFrame;
}

//funkcia pre disparitnu mapu
//filter vstupnych obrazkov, vstupný obrázok, vstupna struktura, vystupny vyfiltrovany obrazok
//ma vplyv na vstupne obrazky, cize aj na vystupnu disparitu
DepthMap DepthMap::filterFrame(DepthMap capture, filterKoef filter)
{
	Mat img = capture.getFrame();

	if (filter.MAX_BLUR_VALUE % 2 == 0 && filter.MAX_BLUR_VALUE != 0)
		filter.MAX_BLUR_VALUE++;
	if (filter.MAX_GAUSS_VALUE % 2 == 0 && filter.MAX_GAUSS_VALUE != 0)
		filter.MAX_GAUSS_VALUE++;
	if (filter.MAX_MEDIAN_VALUE % 2 == 0 && filter.MAX_MEDIAN_VALUE != 0)
		filter.MAX_MEDIAN_VALUE++;
	if (filter.MAX_BILAT_LENGTH % 2 == 0 && filter.MAX_BILAT_LENGTH != 0)
		filter.MAX_BILAT_LENGTH++;
	for (int i = 1; i < filter.MAX_BLUR_VALUE; i = i + 2)
	{
		blur(img, img, Size(i, i), Point(-1, -1));
	}

	/// Applying Gaussian blur
	for (int i = 1; i < filter.MAX_GAUSS_VALUE; i = i + 2)
	{
		GaussianBlur(img, img, Size(i, i), 0, 0);
	}

	/// Applying Median blur
	for (int i = 1; i < filter.MAX_MEDIAN_VALUE; i = i + 2)
	{
		medianBlur(img, img, i);
	}
	/// Applying Bilateral Filter

	for (int i = 1; i < filter.MAX_BILAT_LENGTH; i = i + 2)
	{
		bilateralFilter(img, img, i, i * 2, i / 2);
	}
	capture.setFrame(img);
	return capture;
}

//funkcia pre disparitnu mapu
//vytvorenie disparitnej mapy, vstupne filtrovane obrazky a struktura, vystupna dispatirna mapa
DepthMap DepthMap::stereoCalc(DepthMap captureL, DepthMap captureR, disparityKoef disparity)
{
	DepthMap capture;
	Mat disparityFrame;
	Mat g0 = captureL.getFrame();
	Mat g1 = captureR.getFrame();

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(disparity.vmin + 1, 16 * disparity.vmax, 2 * disparity.smin + 1);
	sgbm->setMinDisparity(disparity.mdip - 50);
	sgbm->setBlockSize(disparity.bsiz + 1);
	sgbm->setP1(disparity.sp1 - 10);
	sgbm->setP2(disparity.sp2 - 10);
	sgbm->setPreFilterCap(disparity.pfc - 10);
	sgbm->setMode(StereoSGBM::MODE_HH);
	sgbm->compute(g0, g1, disparityFrame);
	normalize(disparityFrame, disparityFrame, disparity.alpha, disparity.beta, CV_MINMAX, CV_8U);
	capture.setFrame(disparityFrame);
	return capture;
}

//funkcia pre disparitnu mapu
//filter disparitnej mapy, vstupná disparity, vstupna struktura, vystupny vyfiltrovany obrazok
//vplyv na disparitu, nie na vstupne obrazky
DepthMap DepthMap::resFilterFrame(DepthMap capture, filterKoef filter)
{

	Mat result = capture.getFrame();
	if (filter.RES_BLUR_VALUE % 2 == 0 && filter.RES_BLUR_VALUE != 0)
		filter.RES_BLUR_VALUE++;
	if (filter.RES_GAUSS_VALUE % 2 == 0 && filter.RES_GAUSS_VALUE != 0)
		filter.RES_GAUSS_VALUE++;
	if (filter.RES_MEDIAN_VALUE % 2 == 0 && filter.RES_MEDIAN_VALUE != 0)
		filter.RES_MEDIAN_VALUE++;
	if (filter.RES_BILAT_LENGTH % 2 == 0 && filter.RES_BILAT_LENGTH != 0)
		filter.RES_BILAT_LENGTH++;
	for (int i = 1; i < filter.RES_BLUR_VALUE; i = i + 2)
	{
		blur(result, result, Size(i, i), Point(-1, -1));
	}

	/// Applying Gaussian blur
	for (int i = 1; i < filter.RES_GAUSS_VALUE; i = i + 2)
	{
		GaussianBlur(result, result, Size(i, i), 0, 0);
	}

	/// Applying Median blur
	for (int i = 1; i < filter.RES_MEDIAN_VALUE; i = i + 2)
	{
		medianBlur(result, result, i);
	}

	capture.setFrame(result);
	return capture;
}
