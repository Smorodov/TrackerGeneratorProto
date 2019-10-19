
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "defines.h"
#include "magic_enum.hpp"
#include <cxxproperties.hpp>

class IInput
{
public:
	///
    /// \brief Input type
    ///
    enum Type {None,File,Camera,Points}; 
    ///
    /// \brief BaseTrackerInput
    ///
    IInput() { ; }
    ///
    /// \brief ~BaseTrackerInput
    ///
    ~IInput() { ; }
    ///
    /// \brief Init
    /// \param config
    ///
    virtual bool Init(const cxxproperties::Properties& config) = 0;    
    virtual void GetType(cv::UMat& out)=0;
    virtual void Get(cv::UMat& out)=0;
    ///
    /// \brief PrintListOfProperties
    ///
    virtual void PrintListOfProperties(void) = 0;
};
