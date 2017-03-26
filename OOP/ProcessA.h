
#include "IProcess.h"
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/opencv.hpp"

class ProcessA : public IProcess {
private:
	Mat birdViewCapture;
public:	 
	ProcessA(){
		
	}

	void process(Mat frameLeft, Mat frameRight){
		// birdview prestahovane do processB
	 }

	 Mat getFrame(){
		 return birdViewCapture;
	 }

	 // este prerobit tuto funkciu, zla navratova hodnota
		 Mat getObject() {
		 return birdViewCapture;
	 }

		 ~ProcessA()
		 {}


};