#include "CameraInput.h"

    CameraInput::CameraInput(int cam)
    {
        type = Camera;
        if (!cap.isOpened())
        {
            cap.open(cam);
        }
    }
    CameraInput::CameraInput(std::string fname)
    {
        type = File;
        if (!cap.isOpened())
        {
            cap.open(fname);
        }
    }
    CameraInput::~CameraInput() 
    {
        if (cap.isOpened())
        {
            cap.release();
        }
    }
    void CameraInput::Get(cv::UMat& out)
    {
        if (cap.isOpened())
        {
            cap >> frame;
            if (!frame.empty())
            {
                out = frame.clone();
            }
        }
    }
    bool CameraInput::Init(const cxxproperties::Properties& config)
    {

        return true;
    }    
    void CameraInput::GetType(cv::UMat& out)
    {

    }
    void CameraInput::PrintListOfProperties(void)
    {
        
    }

