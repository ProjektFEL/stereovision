#include "DisparityAlghoritm.h"

DisparityAlghoritm::DisparityAlghoritm(int minDisparity, int blockSize, int p1, int p2, int dispMaxDiff, int preFilterCap, int uniquenessRatio, int speckleWindowSize, int speckleRange, int vMin, int vMax, int sMin)
{
	this->minDisparity = minDisparity;
	this->blockSize = blockSize;
	this->p1 = p1;
	this->p2 = p2;
	this->dispMaxDiff = dispMaxDiff;
	this->preFilterCap = preFilterCap;
	this->uniquenessRatio = uniquenessRatio;
	this->speckleWindowSize = speckleWindowSize;
	this->speckleRange = speckleRange;
	this->vmin = vMin;
	this->vmax = vMax;
	this->smin = sMin;
};

DisparityAlghoritm::DisparityAlghoritm()
{
	vmin = 1, vmax = 3, smin = 0, minDisparity = 15, p1 = 50, p2 = 300, 
	dispMaxDiff = 10, preFilterCap = 10, uniquenessRatio = 30, speckleWindowSize = 10, speckleRange = 10, blockSize = 13;
};

DisparityAlghoritm::~DisparityAlghoritm(){};

int DisparityAlghoritm::getVMin()
{
	return vmin;
};


void DisparityAlghoritm::setVMin(int vmin)
{
	this->vmax = vmin;
};

int DisparityAlghoritm::getVMax() const
{
	return vmax;
};

void DisparityAlghoritm::setVMax(int vmax)
{
	this->vmax = vmax;
};

int DisparityAlghoritm::getSMin() const
{
	return smin;
};

void DisparityAlghoritm::setSMin(int smin)
{
	this->smin = smin;
};

int DisparityAlghoritm::getMinDisparity() const
{
	return minDisparity;
};

void DisparityAlghoritm::setMinDisparity(int minDisparity)
{
	this->minDisparity = minDisparity;
};

int DisparityAlghoritm::getBlockSize() const 
{
	return blockSize;
};

void DisparityAlghoritm::setBlockSize(int blockSize)
{
	this->blockSize = blockSize;
};

int DisparityAlghoritm::getP1() const
{
	return p1;
};

void DisparityAlghoritm::setP1(int p1)
{
	this->p1 = p1;
};

int DisparityAlghoritm::getP2() const
{
	return p2;
};

void DisparityAlghoritm::setP2(int p2)
{
	this->p2 = p2;
};

int DisparityAlghoritm::getDispMaxDiff() const
{
	return dispMaxDiff;
};

void DisparityAlghoritm::setDispMaxDiff(int dispMaxDiff)
{
	this->dispMaxDiff = dispMaxDiff;
};

int DisparityAlghoritm::getPreFilterCap() const
{
	return preFilterCap;
};

void DisparityAlghoritm::setPreFilterCap(int preFilterCap)
{
	this->preFilterCap = preFilterCap;
};

int DisparityAlghoritm::getUniquenessRatio() const
{
	return uniquenessRatio;
};
void DisparityAlghoritm::setUniquenessRatio(int uniquenessRatio)
{
	this->uniquenessRatio = uniquenessRatio;
};

int DisparityAlghoritm::getSpeckleWindowSize() const
{
	return speckleWindowSize;
};

void DisparityAlghoritm::setSpeckleWindowSize(int speckleWindowSize)
{
	this->speckleWindowSize = speckleWindowSize;
};

int DisparityAlghoritm::getSpeckleRange() const
{
	return speckleRange;
};

void DisparityAlghoritm::setSpeckleRange(int speckleRange)
{
	this->speckleRange = speckleRange;
};

Mat DisparityAlghoritm::calculateSGBM(Mat leftImage, Mat rightImage, Mat leftDisparity, Mat rightDisparity)
{
	Mat imgDisparity8U1;
	double minVal, maxVal;

	if (vmax == 0) vmax = 1;

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(vmin + 1, 16 * vmax, 2 * smin + 1);

	sgbm->setMinDisparity(minDisparity - 20);
	sgbm->setBlockSize(blockSize - 10);
	sgbm->setP1(p1 - 10);
	sgbm->setP2(p2 - 10);
	sgbm->setDisp12MaxDiff(dispMaxDiff - 21);
	sgbm->setPreFilterCap(preFilterCap - 10);
	sgbm->setUniquenessRatio(uniquenessRatio - 15);
	sgbm->setSpeckleWindowSize(speckleWindowSize - 10);
	sgbm->setSpeckleRange(speckleRange - 10);
	sgbm->setMode(StereoSGBM::MODE_HH);

	Ptr<StereoMatcher> right_matcher = createRightMatcher(sgbm);
	sgbm->compute(leftImage, rightImage, leftDisparity);
	right_matcher->compute(rightImage, leftImage, rightDisparity);

	getDisparityVis(leftDisparity, imgDisparity8U1, 8); 

	minMaxLoc(leftDisparity, &minVal, &maxVal);

	return (imgDisparity8U1);
};