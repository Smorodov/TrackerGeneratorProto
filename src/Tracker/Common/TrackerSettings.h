#pragma once
#include <iostream>
#include <vector>
#include <memory>
#include <array>
#include <deque>
#include <numeric>
#include "defines.h"
// ----------------------------------------------------------------------
///
/// \brief The TrackerSettings struct
///
class TrackerSettings
{
public:
    //tracking::DistType m_distType;
    tracking::KalmanType m_kalmanType;
    tracking::FilterGoal m_filterGoal;
    tracking::LostTrackType m_lostTrackType;
    tracking::MatchType m_matchType;
	std::array<track_t, tracking::DistsCount> m_distType;
    ///
    /// \brief m_dt
    /// Time step for Kalman
    ///
    track_t m_dt;
    ///
    /// \brief m_accelNoiseMag
    /// Noise magnitude for Kalman
    ///
    track_t m_accelNoiseMag;
    ///
    /// \brief m_distThres
    /// Distance threshold for Assignment problem for tracking::DistCenters or for tracking::DistRects (for tracking::DistJaccard it need from 0 to 1)
    ///
    track_t m_distThres;
    ///
    /// \brief m_maximumAllowedSkippedFrames
    /// If the object don't assignment more than this frames then it will be removed
    ///
    size_t m_maximumAllowedSkippedFrames;
    ///
    /// \brief m_maxTraceLength
    /// The maximum trajectory length
    ///
    size_t m_maxTraceLength;
    ///
    /// \brief m_useAbandonedDetection
    /// Detection abandoned objects
    ///
    bool m_useAbandonedDetection;
    ///
    /// \brief m_minStaticTime
    /// After this time (in seconds) the object is considered abandoned
    ///
    int m_minStaticTime;
    ///
    /// \brief m_maxStaticTime
    /// After this time (in seconds) the abandoned object will be removed
    ///
    int m_maxStaticTime;
	///
	TrackerSettings()
	{
        m_dt = 1.0f;
        m_accelNoiseMag = 0.1f;
        m_distThres = 0.5f;
        m_maximumAllowedSkippedFrames = 25;
        m_maxTraceLength = 50;
        m_useAbandonedDetection = false;
        m_minStaticTime = 5;
        m_maxStaticTime = 25;
        //m_distType = tracking::DistCenters;
        m_kalmanType = tracking::KalmanLinear;
        m_filterGoal = tracking::FilterCenter;
        m_lostTrackType = tracking::TrackKCF;
        m_matchType = tracking::MatchHungrian;

		m_distType[tracking::DistCenters] = 0.0f;
		m_distType[tracking::DistRects] = 0.0f;
		m_distType[tracking::DistJaccard] = 0.5f;
		m_distType[tracking::DistHist] = 0.5f;
		m_distType[tracking::DistHOG] = 0.0f;

		assert(CheckDistance());
	}

	///
	bool CheckDistance() const
	{
		track_t sum = std::accumulate(m_distType.begin(), m_distType.end(), 0.0f);
		track_t maxOne = std::max(1.0f, std::fabs(sum));
		return std::fabs(sum - 1.0f) <= std::numeric_limits<track_t>::epsilon() * maxOne;
	}

	///
	bool SetDistances(std::array<track_t, tracking::DistsCount> distType)
	{
		bool res = true;
		auto oldDists = m_distType;
		m_distType = distType;
		if (!CheckDistance())
		{
			m_distType = oldDists;
			res = false;
		}
		return res;
	}

	///
	bool SetDistance(tracking::DistType distType)
	{
		std::fill(m_distType.begin(), m_distType.end(), 0.0f);
		m_distType[distType] = 1.f;
		return true;
	}
};
