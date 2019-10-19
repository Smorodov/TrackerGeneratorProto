#pragma once
#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <array>
#include "defines.h"
#include "Kalman.h"

// --------------------------------------------------------------------------
///
/// \brief The TrajectoryPoint struct
///
struct TrajectoryPoint
{
    ///
    /// \brief TrajectoryPoint
    ///
    TrajectoryPoint()
        : m_hasRaw(false)
    {
    }

    ///
    /// \brief TrajectoryPoint
    /// \param prediction
    ///
    TrajectoryPoint(const Point_t& prediction)
        :
          m_hasRaw(false),
          m_prediction(prediction)
    {
    }

    ///
    /// \brief TrajectoryPoint
    /// \param prediction
    /// \param raw
    ///
    TrajectoryPoint(const Point_t& prediction, const Point_t& raw)
        :
          m_hasRaw(true),
          m_prediction(prediction),
          m_raw(raw)
    {
    }

    bool m_hasRaw = false;
    Point_t m_prediction;
    Point_t m_raw;
};

// --------------------------------------------------------------------------
///
/// \brief The Trace class
///
class Trace
{
public:
    ///
    /// \brief operator []
    /// \param i
    /// \return
    ///
    const Point_t& operator[](size_t i) const
    {
        return m_trace[i].m_prediction;
    }

    ///
    /// \brief operator []
    /// \param i
    /// \return
    ///
    Point_t& operator[](size_t i)
    {
        return m_trace[i].m_prediction;
    }

    ///
    /// \brief at
    /// \param i
    /// \return
    ///
    const TrajectoryPoint& at(size_t i) const
    {
        return m_trace[i];
    }

    ///
    /// \brief size
    /// \return
    ///
    size_t size() const
    {
        return m_trace.size();
    }

    ///
    /// \brief push_back
    /// \param prediction
    ///
    void push_back(const Point_t& prediction)
    {
        m_trace.emplace_back(prediction);
    }
    void push_back(const Point_t& prediction, const Point_t& raw)
    {
        m_trace.emplace_back(prediction, raw);
    }

    ///
    /// \brief pop_front
    /// \param count
    ///
    void pop_front(size_t count)
    {
        if (count < size())
        {
            m_trace.erase(m_trace.begin(), m_trace.begin() + count);
        }
        else
        {
            m_trace.clear();
        }
    }

    ///
    /// \brief GetRawCount
    /// \param lastPeriod
    /// \return
    ///
    size_t GetRawCount(size_t lastPeriod) const
    {
        size_t res = 0;

        size_t i = 0;
        if (lastPeriod < m_trace.size())
        {
            i = m_trace.size() - lastPeriod;
        }
        for (; i < m_trace.size(); ++i)
        {
            if (m_trace[i].m_hasRaw)
            {
                ++res;
            }
        }

        return res;
    }

private:
    std::deque<TrajectoryPoint> m_trace;
};

// --------------------------------------------------------------------------
///
/// \brief The TrackingObject class
///
struct TrackingObject
{
    cv::RotatedRect m_rrect;
	Trace m_trace;
	size_t m_ID = 0;
	bool m_isStatic = false;
	bool m_outOfTheFrame = false;
	std::string m_type;
	float m_confidence;
	std::vector<cv::Point> m_points;


	///
    TrackingObject(const cv::RotatedRect& rrect, size_t ID, const Trace& trace,
		bool isStatic, bool outOfTheFrame,
		const std::string& type, float confidence)
		:
        m_rrect(rrect), m_ID(ID), m_isStatic(isStatic), m_outOfTheFrame(outOfTheFrame), m_type(type), m_confidence(confidence)
	{
		for (size_t i = 0; i < trace.size(); ++i)
		{
            auto tp = trace.at(i);
            if (tp.m_hasRaw)
            {
                m_trace.push_back(tp.m_prediction, tp.m_raw);
            }
            else
            {
                m_trace.push_back(tp.m_prediction);
            }
		}
	}

	///
	bool IsRobust(int minTraceSize, float minRawRatio, cv::Size2f sizeRatio) const
	{
		bool res = m_trace.size() > static_cast<size_t>(minTraceSize);
		res &= m_trace.GetRawCount(m_trace.size() - 1) / static_cast<float>(m_trace.size()) > minRawRatio;
		if (sizeRatio.width + sizeRatio.height > 0)
		{
            float sr = m_rrect.size.width / m_rrect.size.height;
			if (sizeRatio.width > 0)
			{
				res &= (sr > sizeRatio.width);
			}
			if (sizeRatio.height > 0)
			{
				res &= (sr < sizeRatio.height);
			}
		}
		if (m_outOfTheFrame)
		{
			res = false;
		}
		return res;
	}
};

// --------------------------------------------------------------------------
///
/// \brief The CTrack class
///
class CTrack
{
public:
    CTrack(const CRegion& region,
            tracking::KalmanType kalmanType,
            track_t deltaTime,
            track_t accelNoiseMag,
            size_t trackID,
            bool filterObjectSize,
            tracking::LostTrackType externalTrackerForLost);

    ///
    /// \brief CalcDist
    /// Euclidean distance in pixels between objects centres on two N and N+1 frames
    /// \param reg
    /// \return
    ///
    track_t CalcDistCenter(const CRegion& reg) const;
    ///
    /// \brief CalcDist
    /// Euclidean distance in pixels between object contours on two N and N+1 frames
    /// \param reg
    /// \return
    ///
    track_t CalcDistRect(const CRegion& reg) const;
    ///
    /// \brief CalcDistJaccard
    /// Jaccard distance from 0 to 1 between object bounding rectangles on two N and N+1 frames
    /// \param reg
    /// \return
    ///
    track_t CalcDistJaccard(const CRegion& reg) const;
	///
	/// \brief CalcDistJaccard
	/// Distance from 0 to 1 between objects histogramms on two N and N+1 frames
	/// \param reg
	/// \param currFrame
	/// \return
	///
	track_t CalcDistHist(const CRegion& reg, cv::UMat currFrame) const;
	///
	/// \brief CalcDistHOG
	/// Euclidean distance from 0 to 1 between HOG descriptors on two N and N+1 frames
	/// \param reg
	/// \return
	///
	track_t CalcDistHOG(const CRegion& reg) const;

    bool CheckType(const std::string& type) const;

    void Update(const CRegion& region, bool dataCorrect, size_t max_trace_length, cv::UMat prevFrame, cv::UMat currFrame, int trajLen);

    bool IsStatic() const;
    bool IsStaticTimeout(int framesTime) const;
	bool IsOutOfTheFrame() const;

    cv::RotatedRect GetLastRect() const;

    const Point_t& AveragePoint() const;
    Point_t& AveragePoint();
    const CRegion& LastRegion() const;
    size_t SkippedFrames() const;
    size_t& SkippedFrames();

    TrackingObject ConstructObject() const;

private:
    Trace m_trace;
    size_t m_trackID = 0;
    size_t m_skippedFrames = 0;
    CRegion m_lastRegion;

    Point_t m_predictionPoint;
    cv::RotatedRect m_predictionRect;
    std::unique_ptr<TKalmanFilter> m_kalman;
    bool m_filterObjectSize = false;
    bool m_outOfTheFrame = false;

    tracking::LostTrackType m_externalTrackerForLost;

    void RectUpdate(const CRegion& region, bool dataCorrect, cv::UMat prevFrame, cv::UMat currFrame);

    void CreateExternalTracker(int channels);

    void PointUpdate(const Point_t& pt, const cv::Size& newObjSize, bool dataCorrect, const cv::Size& frameSize);

    bool CheckStatic(int trajLen, cv::UMat currFrame, const CRegion& region);
    bool m_isStatic = false;
    int m_staticFrames = 0;
    cv::UMat m_staticFrame;
    cv::Rect m_staticRect;
};

typedef std::vector<std::unique_ptr<CTrack>> tracks_t;
