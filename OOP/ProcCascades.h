
#include "IProcess.h"


using namespace cv;
using namespace std;

template <typename L> //lines
template <typename S> //sign
class ProcCascades: public IProcess {
private:
	//<L> line;

public:
	ProcCascades()
	{}

	~ProcCascades()
	{}

	virtual void process(Mat rgb, Mat depthMap) 
	{}
	virtual Mat getObject()
	{}
	virtual <L> getLine()
	{}
	virtual <S>[] getSign()
	{}

};