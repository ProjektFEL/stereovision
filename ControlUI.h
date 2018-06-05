#include "IControl.h"
#include "IMotor.h"
#include "MotorA.h"


using namespace boost;
using namespace std;

class ControlUI : public IControl {
private:
IMotor *motorA;
int aB, bB, cB, aS, bS, cS;
int pocitadloPrekazka, pocitadloFrames;

public:

ControlUI(){
motorA = new MotorA();
pocitadloPrekazka = 0;
pocitadloFrames = 0;

	}

	void process(int pUhol, bool withMovement, cv::Mat object, bool brake){

    pocitadloFrames++;
    aS = object.at<int>(0,0);
    bS = object.at<int>(0,1);
    cS = object.at<int>(0,2);
    aB = object.at<int>(0,0);
    bB = object.at<int>(0,1);
    cB = object.at<int>(0,2);

    if(aB == 0 && bB == 0 && cB == 0)
    {
            motorA->process(pUhol,withMovement,1,brake);
    }
    else
    {
        pocitadloPrekazka++;
        if(pocitadloPrekazka > 3)
        {
        motorA->process(pUhol,false,1,true);
        pocitadloPrekazka = 0;
        }
    }
    if(pocitadloFrames>10)
    {
    pocitadloPrekazka--;
    pocitadloFrames = 0;
    }
	}

    int getWheel(){}
	bool getDirection(){}
	int getStrength(){}
	int getBrake(){}
    ~ControlUI(){}
	};
