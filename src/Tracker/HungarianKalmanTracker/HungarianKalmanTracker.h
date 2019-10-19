#pragma once
#include <iostream>
#include <vector>
#include <memory>
#include <array>
#include <deque>
#include <numeric>

#include "defines.h"
#include "track.h"
#include "HungarianAlg.h"
#include "TrackerSettings.h"
#include "ITracker.h"
// ----------------------------------------------------------------------
///
/// \brief The HungarianKalmanTracker class
///
class HungarianKalmanTracker: public ITracker
{
public:
    HungarianKalmanTracker();
    HungarianKalmanTracker(const TrackerSettings& settings);
	HungarianKalmanTracker(const HungarianKalmanTracker&) = delete;
	HungarianKalmanTracker(HungarianKalmanTracker&&) = delete;
	HungarianKalmanTracker& operator=(const HungarianKalmanTracker&) = delete;
	HungarianKalmanTracker& operator=(HungarianKalmanTracker&&) = delete;
	
	~HungarianKalmanTracker(void);

    ///
    /// \brief Init
    /// \param config
    ///
    bool Init(const cxxproperties::Properties& config);

    void Update(const regions_t& regions, cv::UMat currFrame, float fps);

    ///
    /// \brief CanGrayFrameToTrack
    /// \return
    ///
    bool CanGrayFrameToTrack() const
    {
        return true;
    }
	///
	/// \brief CanColorFrameToTrack
	/// \return
	///
	bool CanColorFrameToTrack() const
	{
		return true;
	}
    ///
    /// \brief GetTracksCount
    /// \return
    ///
	size_t GetTracksCount() const
	{
		return m_tracks.size();
	}
    ///
    /// \brief GetTracks
    /// \return
    ///
	std::vector<TrackingObject> GetTracks() const
	{
		std::vector<TrackingObject> tracks;
		if (!m_tracks.empty())
		{
			tracks.reserve(m_tracks.size());
			for (const auto& track : m_tracks)
			{
                tracks.push_back(track->ConstructObject());
			}
		}
		return tracks;
	}

private:
    cxxproperties::Properties config;
    TrackerSettings m_settings;
	tracks_t m_tracks;
    size_t m_nextTrackID;
    cv::UMat m_prevFrame;
    void CreateDistaceMatrix(const regions_t& regions, distMatrix_t& costMatrix, track_t maxPossibleCost, track_t& maxCost, cv::UMat currFrame);
    void SolveHungrian(const distMatrix_t& costMatrix, size_t N, size_t M, assignments_t& assignment);
    void UpdateTrackingState(const regions_t& regions, cv::UMat currFrame, float fps);
    void PrintListOfProperties(void);
};
