
#include <iostream>
#include <vector>
#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"

#include "defines.h"
#include "magic_enum.hpp"
#include <cxxproperties.hpp>
// ----------------------------------------------------------------------
///
/// \brief The IOutput class
///
class IOutput
{
public:
    enum Type {None,Window,File};
    IOutput() {}
    ~IOutput(void) {}
    ///
    /// \brief Init
    /// \param config
    ///
    virtual bool Init(const cxxproperties::Properties& config) = 0;
    ///
    /// \brief Update
    /// \param regions
    /// \param currFrame
    /// \param fps
    ///
    virtual void Update(const regions_t& regions, cv::UMat currFrame, float fps) = 0;
    ///
    /// \brief PrintListOfProperties
    ///
    virtual void PrintListOfProperties(void) = 0;
};
