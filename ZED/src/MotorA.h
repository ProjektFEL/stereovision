#include "IMotor.h"
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

using namespace std;

class MotorA : public IMotor {
private:
int usbdev;
public:

MotorA(){

usbdev=open("/dev/ttyUSB0",O_RDWR);

	}

	~MotorA()
	{}

	void process(int wheel, bool withMovement, int strength, bool brake){

if(wheel >= 0 && wheel <= 60){
sendCommand(usbdev, to_string(wheel), withMovement);
}
	}

	void sendCommand(int usbdev, string wheel, bool withMovement){
	ssize_t bytes_written;
	cout << "Uhol: "<< wheel+" "<<endl;
	if(withMovement)
	{

	char commandForward[4]="";
	commandForward[0]='M';
	if(wheel.length() == 1){
	commandForward[1]='0';
	commandForward[2]=wheel.at(0);
	}
	else {
	commandForward[1]=wheel.at(0);
	commandForward[2]=wheel.at(1);
	}

	commandForward[3]='\n';
	bytes_written = write(usbdev,commandForward,4);

	}else
	{
	char commandServo[4]="";
	commandServo[0]='S';
	if(wheel.length() == 1){
	commandServo[1]='0';
	commandServo[2]=wheel.at(0);
	}
	else {
	commandServo[1]=wheel.at(0);
	commandServo[2]=wheel.at(1);
	}

	commandServo[3]='\n';
	bytes_written = write(usbdev,commandServo,4);
	}

	}

	void goLeft(int usbdev, string wheel)
{

	ssize_t bytes_written;
	char command[4]="";
	command[0]='S';
	command[1]='0';
	command[2]='0';
	command[3]='\n';
	bytes_written = write(usbdev,command,4);
}

void goRight(int usbdev)
{
	ssize_t bytes_written;
	char command[4]="";
	command[0]='S';
	command[1]='6';
	command[2]='0';
	command[3]='\n';
	bytes_written = write(usbdev,command,4);
}

void goMiddle(int usbdev)
{
	ssize_t bytes_written;
	char command[4]="";
	command[0]='S';
	command[1]='3';
	command[2]='7';
	command[3]='\n';
    bytes_written = write(usbdev,command,4);
}

void goForward(int usbdev)
{
	ssize_t bytes_written;
	char command[4]="";
	command[0]='B';
	command[1]='5';
	command[2]='0';
	command[3]='\n';
	bytes_written = write(usbdev,command,4);
}

void noForward(int usbdev)
{
	ssize_t bytes_written;
	char command[4]="";
	command[0]='B';
	command[1]='0';
	command[2]='0';
	command[3]='\n';
	bytes_written = write(usbdev,command,4);
}

	};
