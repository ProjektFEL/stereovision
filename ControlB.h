#include "IControl.h"
#include "IMotor.h"
#include "MotorA.h"



using namespace boost;
using namespace std;

class ControlB : public IControl {
private:
IMotor *motorA;
int aB, bB, cB, aS, bS, cS;


public:

ControlB(){
motorA = new MotorA();

	}

	void process(int pUhol, bool withMovement, cv::Mat object, bool brake){

    aS = object.at<int>(0,0);
    bS = object.at<int>(0,1);
    cS = object.at<int>(0,2);
    aB = object.at<int>(1,0);
    bB = object.at<int>(1,1);
    cB = object.at<int>(1,2);


    if(aB == 0 && bB == 0 && cB == 0)
    {
       if(aS == 0 && bS == 0 && cS == 0)
       {
       motorA->process(30,withMovement,1,brake);
       // vsade volno
       }
       if(aS == 1 && bS == 0 && cS == 0)
       {
       motorA->process(45,withMovement,1,brake);
       // mierne doprava
       }
       if(aS == 0 && bS == 1 && cS == 0)
       {
       motorA->process(45,withMovement,1,brake);
       // mierne doprava
       }
       if(aS == 0 && bS == 0 && cS == 1)
       {
       motorA->process(15,withMovement,1,brake);
       // mierne dolava
       }
       if(aS == 1 && bS == 0 && cS == 1)
       {
       motorA->process(30,withMovement,1,brake);
       // rovno
       }
       if(aS == 1 && bS == 1 && cS == 0)
       {
       motorA->process(50,withMovement,1,brake);
       // mierne doprava
       }
       if(aS == 0 && bS == 1 && cS == 1)
       {
       motorA->process(10,withMovement,1,brake);
       // mierne dolava
       }

    }
    if(aB == 1 && bB == 0 && cB == 0)
    {
       motorA->process(60,withMovement,1,brake);
       // smer doprava
    }
    if(aB == 0 && bB == 1 && cB == 0)
    {
       motorA->process(60,withMovement,1,brake);
       // smer doprava
    }
    if(aB == 0 && bB == 0 && cB == 1)
    {
       motorA->process(0,withMovement,1,brake);
       // smer dolava
    }
    if(aB == 1 && bB == 0 && cB == 1)
    {
       motorA->process(30,withMovement,1,brake);
       // smer rovno
    }
    if(aB == 1 && bB == 1 && cB == 0)
    {
       motorA->process(60,withMovement,1,brake);
       // smer doprava
    }
    if(aB == 0 && bB == 1 && cB == 1)
    {
       motorA->process(0,withMovement,1,brake);
       // smer dolava
    }
    if(aB == 1 && bB == 1 && cB == 1)
    {
       motorA->process(30,false,1,true);
       // vsade prekazka
    }

	}

    int getWheel(){}
	bool getDirection(){}
	int getStrength(){}
	int getBrake(){}
    ~ControlB(){}
	};
