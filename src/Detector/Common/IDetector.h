#pragma once
#include <memory>
#include "defines.h"
#include "magic_enum.hpp"
#include <cxxproperties.hpp>
///
/// \brief The BaseDetector class
///

class IDetector
{
public:
    ///
    /// \brief ~BaseDetector
    ///
    virtual ~IDetector(void)
    {
    }
    ///
    /// \brief Init
    /// \param config
    ///
    virtual bool Init(const cxxproperties::Properties& config) = 0;
    ///
    /// \brief Detect
    /// \param frame
    ///
    virtual void Detect(cv::UMat& frame) = 0;
	///
	/// \brief CanGrayProcessing
	///
	virtual bool CanGrayProcessing(void) const = 0;
    ///
    /// \brief PrintListOfProperties
    ///
    virtual void PrintListOfProperties(void) = 0;
    ///
    /// \brief Get
    ///
    virtual regions_t Get(void) = 0;
};
