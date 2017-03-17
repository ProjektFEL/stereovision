#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"


#include <stdio.h>
#include <string>
#include <iostream>

#ifndef STRUCTURES_H_INCLUDED
#define STRUCTURES_H_INCLUDED
using namespace cv;
using namespace std;


typedef struct disparityKoef
{
    int vmin=1, vmax =3, smin = 4, mdip = 15, ndip = 10, sp1 = 50, sp2 = 300;
    int dmd = 10, pfc = 10, sur = 30, sws = 10, ssr = 10, sm = 10, bsiz = 13;
    int rangeDispLow=0,rangeDispHigh=255, alpha=0, beta=255;
    disparityKoef(){};

    void readDisparityKoef();
    void writeDisparityKoef();
    void setDisparityKoef();

} disparityKoef;

typedef struct filterKoef
{
    int MAX_BLUR_VALUE=0, MAX_GAUSS_VALUE=0, MAX_MEDIAN_VALUE=0, MAX_BILAT_LENGTH=0;
    int RES_BLUR_VALUE=0, RES_GAUSS_VALUE=0, RES_MEDIAN_VALUE=0, RES_BILAT_LENGTH=0;
    filterKoef(){};

    void readFilterKoef();
    void writeFilterKoef();
    void setFilterKoef();

} filterKoef;

typedef struct rotatKoef
{
    Mat  M1, D1, M2, D2;
    Mat  R1, P1, R2, P2;

    rotatKoef(){};

    void readRotatKoef();

} rotatKoef;

typedef struct laneAssistKoef
{

    int ptX1=80,ptX2=80,ptY1=145,ptY2=145,gray=15;
    int theta1=56,rho1=0,theta2=19,rho2=0,max_lenght1=24,max_lenght2=27,edmin=2,edmax=255;

    laneAssistKoef(){};

    void readLaneAssistKoef();
    void writeLaneAssistKoef();
    void setLaneAssistKoef();


}laneAssistKoef;


#endif // STRUCTURES_H_INCLUDED
