
#include "IControl.h"

template <typename L,typename S>
class ConBasic : public IControl{
public:
	ConBasic()
	{}
	
	~ConBasic()
	{}

	virtual void process(Mat object, T line, S[] signs)
	{}

	virtual int getWheel()
	{}

	virtual bool getDirection()
	{}

	virtual int getStrength()
	{}

	virtual int getBrake()
	{}

};