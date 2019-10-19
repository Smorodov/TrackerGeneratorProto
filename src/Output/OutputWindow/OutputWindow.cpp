#include "OutputWindow.h"
///
/// \brief Init
/// \param config
///
bool OutputWindow::Init(const cxxproperties::Properties& config)
{
    return true;
}
///
/// \brief Update
/// \param regions
/// \param currFrame
/// \param fps
///
void OutputWindow::Update(const regions_t& regions, cv::UMat currFrame, float fps)
{
    for (int i = 0; i < regions.size(); ++i)
    {
        cv::rectangle(currFrame, regions[i].m_rrect.boundingRect(), cv::Scalar::all(255), 1);
    }
    cv::imshow("Output", currFrame);
    cv::waitKey(1);
}
///
/// \brief PrintListOfProperties
///
void OutputWindow::PrintListOfProperties(void)
{

}
