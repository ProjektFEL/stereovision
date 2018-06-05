
#include "IProcess.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctime>
#include <sstream>

using namespace std;

namespace patch
{
    template < typename T > std::string to_string(const T& n)
    {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
}
typedef unsigned char byte;
//kluc na oddelenie rozmerov timeStamp imageData
string delim ="break";

class ProcessPython : public IProcess {
private:
    thread* t;
	cv::Mat copyLeft, copyRight, copyDisparity, sobel, sobely;
	cv::Mat image;
	//funkcia na ziskanie aktualneho casu
std::string now()
{
    std::time_t tt = std::time(NULL);
    std::string s  = std::ctime(&tt);
    return s.substr(0,s.size()-1);
}

//funkcia na prevod Mat obrazka do vektora typu byte = unsigned char
std::vector<byte> matToBytes(cv::Mat image)
{
    int size = image.total() * image.elemSize();
    std::vector<byte> img_bytes(size);
    img_bytes.assign(image.datastart, image.dataend);
    return img_bytes;
}


public:
	ProcessPython(){
    //system("python ~//Documents//ZED/src//python//predictions.py");
	}

	~ProcessPython()
	{}

thread* run(mutex* z, cv::Mat frameLeft, cv::Mat frameRight) {
		// z->lock();
		frameLeft.copyTo(copyLeft);
		frameRight.copyTo(copyRight);
		// z->unlock();
        t = new thread(&ProcessPython::process, this, copyLeft, copyRight);
		return t;
	}


	void work(cv::Mat frameLeft, cv::Mat frameRight){
		for (int i = 1; i <= 100; i++)
		{
			cout << "ProcessPython: " << i << endl;
		}
	}


	void process(cv::Mat frameLeft, cv::Mat backgroundImage){

// create pipe
    int error=mkfifo("/tmp/fifo", S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
    // check for errors
    if(error!=0){
     // file exists
       if(errno!=EEXIST){
         // write error
         cout<<strerror(errno)<<endl;

       }
     }
     cout<<"Created"<<endl;

     // open for write
     int pajpa_w;
     if((pajpa_w=open("/tmp/fifo", O_WRONLY))<0){
         cout<<strerror(errno)<<endl;

     }
     cout<<"Opened"<<endl;



    // nacitanie obrazka
    frameLeft.copyTo(image);
frameLeft.convertTo(frameLeft, CV_8UC1);




    //ulozenie rozmerov obrazka do premennych
    std::string width = patch::to_string(image.size().width) +delim;
    std::string height = patch::to_string(image.size().height) +delim;
    std::string depth = patch::to_string(image.channels()) +delim;


    //zapis rozmerov obrazka do pajpy
    if ((error=write(pajpa_w, width.c_str(), width.length())) < 0){

        cout<<strerror(errno)<<endl;

    };
      if ((error=write(pajpa_w, height.c_str(), height.length())) < 0){

        cout<<strerror(errno)<<endl;

    };
      if ((error=write(pajpa_w, depth.c_str(), depth.length())) < 0){

        cout<<strerror(errno)<<endl;

    };


    //vytvorenie timeStamp + na konci je break podla toho sa v pythone vie oddelit timeStamp od imageData

    string timeStamp = now()+delim;

    //cout <<timeStamp<< std::endl;


    //prevod obrazka na vektor

    std::vector<byte> data;

    data = matToBytes(image);

    //cout << ("Image data size: %lu bytes", data.size()) << endl;

    //zapis dataStamp do pajpy
    if ((error=write(pajpa_w, timeStamp.c_str(), timeStamp.length())) < 0){

        cout<<strerror(errno)<<endl;

    };

    //zapis imageData do pajpy
    if ((error=write(pajpa_w, data.data(), data.size())) < 0) {

        cout<<strerror(errno)<<endl;

    }

    cout<<"Written"<<endl;

    // close pipe
    close(pajpa_w);

    //otvorenie pajpy
     const char *fifo_r = "/tmp/fifo1";

     mknod(fifo_r, S_IFIFO | 0666, 0);
     std::ifstream f(fifo_r);

     // prijate data
     std::string receivedTimestamp;
     std::string receivedResult;

     getline(f, receivedTimestamp);
     getline(f, receivedResult);

     cout<<receivedTimestamp<<endl;
     cout<<receivedResult<<endl;

	 }

	cv::Mat getFrame(){
		 return copyLeft;
	 }

	 // este prerobit tuto funkciu, zla navratova hodnota
	cv::Mat getObject() {
			 return copyLeft;
	 }
	 double getAngle(){
	return 0;
	}

	cv::Mat getDisparity(){
		 return copyLeft;
	 }

};
