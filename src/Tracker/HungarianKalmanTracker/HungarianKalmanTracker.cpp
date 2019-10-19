#include "ITracker.h"
#include "HungarianKalmanTracker.h"
#include <magic_enum.hpp>

HungarianKalmanTracker::HungarianKalmanTracker() :
    m_nextTrackID(0)
{

}
///
/// \brief HungarianKalmanTracker::HungarianKalmanTracker
/// Tracker. Manage tracks. Create, remove, update.
/// \param settings
///
HungarianKalmanTracker::HungarianKalmanTracker(const TrackerSettings& settings)
    :
      m_settings(settings),
      m_nextTrackID(0)
{
}

///
/// \brief CTracker::~CTracker
///
HungarianKalmanTracker::~HungarianKalmanTracker(void)
{
}

///
/// \brief HungarianKalmanTracker::Update
/// \param regions
/// \param currFrame
/// \param fps
///
void HungarianKalmanTracker::Update(
        const regions_t& regions,
        cv::UMat currFrame,
        float fps
        )
{
    UpdateTrackingState(regions, currFrame, fps);
    currFrame.copyTo(m_prevFrame);
}

///
/// \brief HungarianKalmanTracker::UpdateTrackingState
/// \param regions
/// \param currFrame
/// \param fps
///
void HungarianKalmanTracker::UpdateTrackingState(
        const regions_t& regions,
        cv::UMat currFrame,
        float fps
        )
{
    const size_t N = m_tracks.size();	// Tracking objects
    const size_t M = regions.size();	// Detections or regions

    assignments_t assignment(N, -1); // Assignments regions -> tracks

    if (!m_tracks.empty())
    {
        // Distance matrix between all tracks to all regions
        distMatrix_t costMatrix(N * M);
        const track_t maxPossibleCost = static_cast<track_t>(currFrame.cols * currFrame.rows);
        track_t maxCost = 0;
        CreateDistaceMatrix(regions, costMatrix, maxPossibleCost, maxCost, currFrame);
        SolveHungrian(costMatrix, N, M, assignment);
        // clean assignment from pairs with large distance
        for (size_t i = 0; i < assignment.size(); i++)
        {
            if (assignment[i] != -1)
            {
                if (costMatrix[i + assignment[i] * N] > m_settings.m_distThres)
                {
                    assignment[i] = -1;
                    m_tracks[i]->SkippedFrames()++;
                }
            }
            else
            {
                // If track have no assigned detect, then increment skipped frames counter.
                m_tracks[i]->SkippedFrames()++;
            }
        }

        // If track didn't get detects long time, remove it.
        for (int i = 0; i < static_cast<int>(m_tracks.size()); i++)
        {
            if (m_tracks[i]->SkippedFrames() > m_settings.m_maximumAllowedSkippedFrames ||
                    m_tracks[i]->IsStaticTimeout(cvRound(fps * (m_settings.m_maxStaticTime - m_settings.m_minStaticTime))))
            {
                m_tracks.erase(m_tracks.begin() + i);
                assignment.erase(assignment.begin() + i);
                i--;
            }
        }
    }

    // Search for unassigned detects and start new tracks for them.
    for (size_t i = 0; i < regions.size(); ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
            m_tracks.push_back(std::make_unique<CTrack>(regions[i],
                                                      m_settings.m_kalmanType,
                                                      m_settings.m_dt,
                                                      m_settings.m_accelNoiseMag,
                                                      m_nextTrackID++,
                                                      m_settings.m_filterGoal == tracking::FilterRect,
                                                      m_settings.m_lostTrackType));
        }
    }

    // Update Kalman Filters state
    const ptrdiff_t stop_i = static_cast<ptrdiff_t>(assignment.size());
#pragma omp parallel for
    for (ptrdiff_t i = 0; i < stop_i; ++i)
    {
        // If track updated less than one time, than filter state is not correct.
        if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
        {
            m_tracks[i]->SkippedFrames() = 0;
            m_tracks[i]->Update(
                        regions[assignment[i]], true,
                    m_settings.m_maxTraceLength,
                    m_prevFrame, currFrame,
                    m_settings.m_useAbandonedDetection ? cvRound(m_settings.m_minStaticTime * fps) : 0);
        }
        else				     // if not continue using predictions
        {
            m_tracks[i]->Update(CRegion(), false, m_settings.m_maxTraceLength, m_prevFrame, currFrame, 0);
        }
    }
}

///
/// \brief HungarianKalmanTracker::CreateDistaceMatrix
/// \param regions
/// \param costMatrix
/// \param maxPossibleCost
/// \param maxCost
///
void HungarianKalmanTracker::CreateDistaceMatrix(const regions_t& regions, distMatrix_t& costMatrix, track_t maxPossibleCost, track_t& maxCost, cv::UMat currFrame)
{
    const size_t N = m_tracks.size();	// Tracking objects
    maxCost = 0;

	for (size_t i = 0; i < m_tracks.size(); ++i)
	{
		const auto& track = m_tracks[i];

		for (size_t j = 0; j < regions.size(); ++j)
		{
			auto dist = maxPossibleCost;
			if (m_tracks[i]->CheckType(regions[j].m_type))
			{
				dist = 0;
				size_t ind = 0;
				if (m_settings.m_distType[ind] > 0.0f && ind == tracking::DistCenters)
				{
					dist += m_settings.m_distType[ind] * track->CalcDistCenter(regions[j]);
				}
				++ind;
				if (m_settings.m_distType[ind] > 0.0f && ind == tracking::DistRects)
				{
					dist += m_settings.m_distType[ind] * track->CalcDistRect(regions[j]);
				}
				++ind;
				if (m_settings.m_distType[ind] > 0.0f && ind == tracking::DistJaccard)
				{
					dist += m_settings.m_distType[ind] * track->CalcDistJaccard(regions[j]);
				}
				++ind;
				if (m_settings.m_distType[ind] > 0.0f && ind == tracking::DistHist)
				{
					dist += m_settings.m_distType[ind] * track->CalcDistHist(regions[j], currFrame);
				}
				++ind;
				if (m_settings.m_distType[ind] > 0.0f && ind == tracking::DistHOG)
				{
					dist += m_settings.m_distType[ind] * track->CalcDistHOG(regions[j]);
				}
				++ind;
				assert(ind == tracking::DistsCount);
			}

			costMatrix[i + j * N] = dist;
			if (dist > maxCost)
			{
				maxCost = dist;
			}
		}
	}
}

///
/// \brief HungarianKalmanTracker::SolveHungrian
/// \param costMatrix
/// \param N
/// \param M
/// \param assignment
///
void HungarianKalmanTracker::SolveHungrian(const distMatrix_t& costMatrix, size_t N, size_t M, assignments_t& assignment)
{
    AssignmentProblemSolver APS;
    APS.Solve(costMatrix, N, M, assignment, AssignmentProblemSolver::optimal);
}

///
/// \brief Init
/// \param config
///
bool HungarianKalmanTracker::Init(const cxxproperties::Properties& config)
{
    this->config = config;

    m_settings.m_dt = config.get<float>("dt", 1.0f);
    m_settings.m_accelNoiseMag = config.get<float>("accelNoiseMag", 1.0f);
    m_settings.m_distThres = config.get<float>("distThres", 0.5f);
    m_settings.m_maximumAllowedSkippedFrames = config.get<int>("maximumAllowedSkippedFrames", 25);
    m_settings.m_maxTraceLength = config.get<int>("maxTraceLength", 50);
    m_settings.m_useAbandonedDetection = config.get<bool>("useAbandonedDetection", false);
    m_settings.m_minStaticTime = config.get<int>("minStaticTime", 5);
    m_settings.m_maxStaticTime = config.get<int>("maxStaticTime", 25);
    //m_settings.m_distType = tracking::DistCenters;
    m_settings.m_kalmanType = magic_enum::enum_cast<tracking::KalmanType>(config.get<std::string>("kalmanType",static_cast<const std::string>(magic_enum::enum_name(tracking::KalmanLinear)))).value();
    m_settings.m_filterGoal = magic_enum::enum_cast<tracking::FilterGoal>(config.get<std::string>("filterGoal", static_cast<const std::string>(magic_enum::enum_name(tracking::FilterRect)))).value();
    m_settings.m_lostTrackType = magic_enum::enum_cast<tracking::LostTrackType>(config.get<std::string>("lostTrackType", static_cast<const std::string>(magic_enum::enum_name(tracking::TrackKCF)))).value();
    m_settings.m_matchType = magic_enum::enum_cast<tracking::MatchType>(config.get<std::string>("matchType", static_cast<const std::string>(magic_enum::enum_name(tracking::MatchHungrian)))).value();
    m_settings.m_distType[tracking::DistCenters] = config.get<float>("distType_DistCenters", 0.0f);
    m_settings.m_distType[tracking::DistRects] = config.get<float>("distType_DistRects", 0.0f);
    m_settings.m_distType[tracking::DistJaccard] = config.get<float>("distType_DistJaccard", 0.5f);
    m_settings.m_distType[tracking::DistHist] = config.get<float>("distType_DistHist", 0.5f);
    m_settings.m_distType[tracking::DistHOG] = config.get<float>("distType_DistHOG", 0.0f);

    // TODO: append more props

    return true;
}

void HungarianKalmanTracker::PrintListOfProperties(void)
{
    // Print all pairs key/value (values are returned as std::string by default)
    std::cout << "Entries (returned as string): " << std::endl;
    for (const auto& key : config.get_keys()) {
        std::cout << "  * " << key << " = " << config.get(key) << std::endl;
    }
    std::cout << std::endl;
}