#pragma once
#include <iostream>
#include <vector>
#include <memory>
#include <array>
#include <deque>
#include <numeric>

#include "defines.h"
#include "track.h"
#include "magic_enum.hpp"
#include <cxxproperties.hpp>
// ----------------------------------------------------------------------
///
/// \brief The CTracker class
///
class ITracker
{
public:
    ITracker() {}
    ~ITracker(void) {}
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
    virtual void Update(const regions_t& regions, cv::UMat currFrame, float fps)=0;
    ///
    /// \brief CanGrayFrameToTrack
    /// \return
    ///
    virtual bool CanGrayFrameToTrack(void) const = 0;
	///
	/// \brief CanColorFrameToTrack
	/// \return
	///
    virtual bool CanColorFrameToTrack(void) const = 0;
    ///
    /// \brief GetTracksCount
    /// \return
    ///
    virtual size_t GetTracksCount() const = 0;
    ///
    /// \brief GetTracks
    /// \return
    ///
    virtual std::vector<TrackingObject> GetTracks() const = 0;
    ///
    /// \brief PrintListOfProperties
    ///
    virtual void PrintListOfProperties(void) = 0;
};
