
#include "ICapture.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

class CapZEN3D: public ICapture {
public:
	
	Mat   frame[2],display[2];
	Mat rgb, depth, object;
	VideoCapture capture[2];
	

	// parameters are videoFile(path to file) 
	CapZEN3D(string pPath1, string pPath2)
	{
		capture[0].open(pPath1);
		capture[1].open(pPath2);
	}
	// parameters are usb input(number)
	CapZEN3D(int pPath1, int pPath2)
	{
		capture[0].open(pPath1);
		capture[1].open(pPath2);
	}

	~CapZEN3D()
	{}

	virtual void process()
	{
		omp_set_num_threads(2);
#pragma omp parallel
			{
				int TID = omp_get_thread_num();
				capture[TID].read(frame[TID]);
				if (frame[TID].empty()) {
					cout << "Empty frame" + TID << endl;
					exit(0);
				}
				erode(frame[TID], frame[TID], getStructuringElement(MORPH_RECT, Size(3, 3)));
				dilate(frame[TID], frame[TID], getStructuringElement(MORPH_RECT, Size(3, 3)));
				frame[TID].copyTo(display[TID]);
#pragma omp barrier
			}
	}

	virtual Mat getLeftRGB()
	{
		return frame[0];
	}
	virtual Mat getRightRGB()
	{
		return frame[1];
	}

	virtual Mat getDepthMap()
	{
		Mat pom;
		return pom;
	}

};