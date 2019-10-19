#include "IInput.h"

class CameraInput: public IInput
{
public:    
    CameraInput(int cam);
    CameraInput(std::string fname);
    ~CameraInput();
    void Get(cv::UMat& out);
    bool Init(const cxxproperties::Properties& config);    
    void GetType(cv::UMat& out);
    void PrintListOfProperties(void);
private:
    Type type;
    cv::VideoCapture cap;    
    cv::UMat frame;
};
