#pragma once

#include "IDetector.h"

///
/// \brief The FaceDetector class
///
class FaceDetector : public IDetector
{
public:
    FaceDetector();
    ~FaceDetector(void);
    cxxproperties::Properties config;
    bool Init(const cxxproperties::Properties& config);

    void Detect(cv::UMat& gray);

	bool CanGrayProcessing(void) const
	{
		return true;
	}

    void PrintListOfProperties(void);
    regions_t Get(void);
private:
    cv::CascadeClassifier m_cascade;
    cv::Size m_minObjectSize;
    regions_t m_regions;
};
