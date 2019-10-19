#include "FaceDetector.h"

///
/// \brief FaceDetector::FaceDetector
/// \param gray
///
FaceDetector::FaceDetector()
{
}

///
/// \brief FaceDetector::~FaceDetector
///
FaceDetector::~FaceDetector(void)
{
}
void FaceDetector::PrintListOfProperties(void)
{
    // Print all pairs key/value (values are returned as std::string by default)
    std::cout << "Entries (returned as string): " << std::endl;
    for (const auto& key : config.get_keys()) {
        std::cout << "  * " << key << " = " << config.get(key) << std::endl;
    }
    std::cout << std::endl;
}
///
/// \brief FaceDetector::Get
/// \return
///
regions_t FaceDetector::Get(void)
{
    return m_regions;
}

///
/// \brief FaceDetector::Init
/// \param cascadeFileName
/// \return
///
bool FaceDetector::Init(const cxxproperties::Properties& config)
{
    this->config = config;

    if (config.contains("cascadeFileName"))
    {
        std::string cascadeFileName = config.get<std::string>("cascadeFileName");
        if (!m_cascade.load(cascadeFileName) || m_cascade.empty())
        {
        std::cerr << "Cascade " << cascadeFileName << " not opened!" << std::endl;
        return false;
        }

    }else
    {
        std::cerr << " \"cascadeFileName\" key not found in config !" << std::endl;
        return false;
    }
    m_minObjectSize = cv::Size(config.get<int>("minObjectWidth", 100), config.get<int>("minObjectHeight", 100));
    return true;
}

///
/// \brief FaceDetector::Detect
/// \param gray
///
void FaceDetector::Detect(cv::UMat& gray)
{
    bool findLargestObject = false;
    bool filterRects = true;
    std::vector<cv::Rect> faceRects;
    m_cascade.detectMultiScale(gray,
                             faceRects,
                             1.1,
                             (filterRects || findLargestObject) ? 3 : 0,
                             findLargestObject ? cv::CASCADE_FIND_BIGGEST_OBJECT : 0,
                             m_minObjectSize,
                             cv::Size(gray.cols / 2, gray.rows / 2));
    m_regions.clear();
    for (auto rect : faceRects)
    {
        m_regions.push_back(rect);
    }
}
