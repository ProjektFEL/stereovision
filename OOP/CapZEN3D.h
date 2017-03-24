
#include "ICapture.h"

class CapZEN3D: public ICapture {
public:
	CapZEN3D()
	{}

	~CapZEN3D()
	{}

	virtual void process()
	{}

	virtual Mat getRGB()
	{
		Mat pom;
		return pom;
	}

	virtual Mat getDepthMap()
	{
		Mat pom;
		return pom;
	}

};