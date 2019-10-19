#include "IOutput.h"

// ----------------------------------------------------------------------
///
/// \brief The IOutput class
///
class OutputWindow : public IOutput
{
public:
    OutputWindow() {}
    ~OutputWindow(void) {}
    ///
    /// \brief Init
    /// \param config
    ///
    bool Init(const cxxproperties::Properties& config);
    ///
    /// \brief Update
    /// \param regions
    /// \param currFrame
    /// \param fps
    ///
    void Update(const regions_t& regions, cv::UMat currFrame, float fps);
    ///
    /// \brief PrintListOfProperties
    ///
    void PrintListOfProperties(void);

private:
    Type type;
};
