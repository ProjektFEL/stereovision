#include "Capture.h"
#include "structures.h"

using namespace cv;
using namespace std;


void disparityKoef::readDisparityKoef()
{
    FileStorage fs("disparity.yml", FileStorage::READ);
    fs["vmin"]>> vmin;
    fs["vmax"]>> vmax;
    fs["mdip"]>> mdip;
    fs["bsiz"]>> bsiz;
    fs["sp1"] >> sp1;
    fs["sp2"] >> sp2;
    fs["pfc"] >> pfc;
    fs["rangeDispLow"] >> rangeDispLow;
    fs["rangeDispHigh"]>> rangeDispHigh;
    fs["alpha"] >> alpha;
    fs["beta"]>> beta;
    fs.release();
}

void disparityKoef::writeDisparityKoef()
{

    FileStorage fs("disparity.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "vmin" << vmin << "vmax" << vmax <<"mdip" << mdip << "bsiz" << bsiz <<"sp1"<<sp1<<"sp2"<<sp2<<"pfc"<<pfc
        <<"alpha"<<alpha<<"beta"<<beta<<"rangeDispLow"<<rangeDispLow<<"rangeDispHigh"<<rangeDispHigh;
        fs.release();
    }
}

void disparityKoef::setDisparityKoef()
{
    cvNamedWindow("StereoSGBM control", 0);
    cvCreateTrackbar("Vmin", "StereoSGBM control", &vmin, 99, 0);
    cvCreateTrackbar("Vmax", "StereoSGBM control", &vmax, 15, 0);
    cvCreateTrackbar("mdip", "StereoSGBM control", &mdip, 99, 0);
    cvCreateTrackbar("bsiz", "StereoSGBM control", &bsiz, 99, 0);
    cvCreateTrackbar("sp1", "StereoSGBM control", &sp1, 1000, 0);
    cvCreateTrackbar("sp2", "StereoSGBM control", &sp2, 5000, 0);
    cvCreateTrackbar("pfc", "StereoSGBM control", &pfc, 200, 0);
    cvCreateTrackbar("alpha", "StereoSGBM control", &alpha, 300, 0);
    cvCreateTrackbar("beta", "StereoSGBM control", &beta, 300, 0);
    cvCreateTrackbar("L1", "StereoSGBM control", &rangeDispLow, 255, 0);
    cvCreateTrackbar("L2", "StereoSGBM control", &rangeDispHigh, 255, 0);

    if(rangeDispHigh<=rangeDispLow) rangeDispHigh=rangeDispLow+1;

    if(vmax==0) vmax=1;
}

void filterKoef::readFilterKoef()
{
    FileStorage fs("filter.yml", FileStorage::READ);
    fs["MAX_BLUR_VALUE"]>> MAX_BLUR_VALUE;
    fs["MAX_GAUSS_VALUE"]>> MAX_GAUSS_VALUE;
    fs["MAX_MEDIAN_VALUE"]>> MAX_MEDIAN_VALUE;
    fs["MAX_BILAT_LENGTH"]>> MAX_BILAT_LENGTH;
    fs["RES_BLUR_VALUE"] >> RES_BLUR_VALUE;
    fs["RES_GAUSS_VALUE"] >> RES_GAUSS_VALUE;
    fs["RES_MEDIAN_VALUE"] >> RES_MEDIAN_VALUE;
    fs["RES_BILAT_LENGTH"]>> RES_BILAT_LENGTH;
    fs.release();
}

void filterKoef::writeFilterKoef()
{

    FileStorage fs("filter.yml", FileStorage::WRITE);

    if( fs.isOpened() )
    {
        fs << "MAX_BLUR_VALUE" << MAX_BLUR_VALUE << "MAX_GAUSS_VALUE" << MAX_GAUSS_VALUE <<"MAX_MEDIAN_VALUE" << MAX_MEDIAN_VALUE <<
        "MAX_BILAT_LENGTH" << MAX_BILAT_LENGTH <<"RES_BLUR_VALUE"<<RES_BLUR_VALUE<<"RES_GAUSS_VALUE"<<RES_GAUSS_VALUE<<
        "RES_MEDIAN_VALUE"<<RES_MEDIAN_VALUE<<"RES_BILAT_LENGTH"<<RES_BILAT_LENGTH;
        fs.release();
    }
}

void filterKoef::setFilterKoef()
{
    cvNamedWindow("Filters", 0);
    cvCreateTrackbar("blur", "Filters", &MAX_BLUR_VALUE, 50, 0);
    cvCreateTrackbar("gauss", "Filters", &MAX_GAUSS_VALUE, 50, 0);
    cvCreateTrackbar("median", "Filters", &MAX_MEDIAN_VALUE, 50, 0);
    cvCreateTrackbar("resBlur", "Filters", &RES_BLUR_VALUE, 50, 0);
    cvCreateTrackbar("resGauss", "Filters", &RES_GAUSS_VALUE, 50, 0);
    cvCreateTrackbar("resMedian", "Filters", &RES_MEDIAN_VALUE, 50, 0);
}

void rotatKoef::readRotatKoef()
{
    FileStorage fs1("intrinsics.yml", CV_STORAGE_READ);
    fs1["M1"] >> M1;
    fs1["M2"] >> M2;
    fs1["D1"] >> D1;
    fs1["D2"] >> D2;

    fs1.release();

    FileStorage fs2("extrinsics.yml", CV_STORAGE_READ);
    fs2["R1"] >> R1;
    fs2["R2"] >> R2;
    fs2["P1"] >> P1;
    fs2["P2"] >> P2;

    fs2.release();
}

void laneAssistKoef::readLaneAssistKoef()
{
    FileStorage fs("laneAssist.yml", FileStorage::READ);
    //bird view
    fs["gray"]>> gray;
    fs["ptX1"]>> ptX1;
    fs["ptX2"]>> ptX2;
    fs["ptY1"]>> ptY1;
    fs["ptY2"] >> ptY2;
    //lane detect
    fs["theta1"] >> theta1;
    fs["theta2"] >> theta2;
    fs["rho1"] >> rho1;
    fs["rho2"]>> rho2;
    fs["max_lenght1"] >> max_lenght1;
    fs["max_lenght2"]>> max_lenght2;

    fs.release();
}

void laneAssistKoef::writeLaneAssistKoef()
{
    FileStorage fs("laneAssist.yml", FileStorage::WRITE);

    if( fs.isOpened() )
    {
        fs << "gray" << gray << "ptX1" << ptX1 <<"ptX2" << ptX2 << "ptY1" << ptY1 <<"ptY2"<<ptY2<<"theta1"<<theta1<<
        "theta2"<<theta2<<"rho1"<<rho1<<"rho2"<<rho2<<"max_lenght1"<<max_lenght1<<"max_lenght2"<<max_lenght2;

        fs.release();
    }
}

void laneAssistKoef::setLaneAssistKoef()
{
    cvNamedWindow("LaneAssist control", CV_WINDOW_NORMAL);
    cvCreateTrackbar("gray", "LaneAssist control", &gray, 255);
    cvCreateTrackbar("ptX1", "LaneAssist control", &ptX1, 320);
    cvCreateTrackbar("ptY1", "LaneAssist control", &ptY1, 240);
    cvCreateTrackbar("ptX2", "LaneAssist control", &ptX2, 320);
    cvCreateTrackbar("ptY2", "LaneAssist control", &ptY2, 240);
    cvCreateTrackbar("theta1","LaneAssist control", &theta1, 360);
    cvCreateTrackbar("rho1", "LaneAssist control", &rho1, 360);
    cvCreateTrackbar("line lenght1", "LaneAssist control", &max_lenght1, 500);
    cvCreateTrackbar("theta2", "LaneAssist control", &theta2, 500);
    cvCreateTrackbar("rho2", "LaneAssist control", &rho2, 500);
    cvCreateTrackbar("line lenght2", "LaneAssist control", &max_lenght2, 500);
}
