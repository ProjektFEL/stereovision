#include "IControl.h"
#include "IMotor.h"
#include "MotorA.h"


using namespace boost;
using namespace std;

class ControlA : public IControl {
private:
IMotor *motorA;


public:

ControlA(){
motorA = new MotorA();

	}

	void process(int pUhol, bool withMovement){

motorA->process(pUhol,withMovement,1,false);

	}

    int getWheel(){}
	bool getDirection(){}
	int getStrength(){}
	int getBrake(){}
    ~ControlA(){}
	};
