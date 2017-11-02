
#include "IProcess.h"



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

	virtual void process(cv::Mat rgb, cv::Mat depthMap)
	{}
	virtual cv::Mat getObject()
	{}
	virtual <L> getLine()
	{}
	virtual <S>[] getSign()
	{}

};