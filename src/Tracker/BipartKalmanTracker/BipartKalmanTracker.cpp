#include "Ctracker.h"

#include <GTL/GTL.h>
 #include "mygraph.h"
 #include "mwbmatching.h"
 #include "tokenise.h"

///
/// \brief CTracker::CTracker
/// Tracker. Manage tracks. Create, remove, update.
/// \param settings
///
CTracker::CTracker(const TrackerSettings& settings)
    :
      m_settings(settings),
      m_nextTrackID(0)
{
}

///
/// \brief CTracker::~CTracker
///
CTracker::~CTracker(void)
{
}

///
/// \brief CTracker::Update
/// \param regions
/// \param currFrame
/// \param fps
///
void CTracker::Update(
        const regions_t& regions,
        cv::UMat currFrame,
        float fps
        )
{
    UpdateTrackingState(regions, currFrame, fps);

    currFrame.copyTo(m_prevFrame);
}

///
/// \brief CTracker::UpdateTrackingState
/// \param regions
/// \param currFrame
/// \param fps
///
void CTracker::UpdateTrackingState(
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

        // Solving assignment problem (tracks and predictions of Kalman filter)
        if (m_settings.m_matchType == tracking::MatchHungrian)
        {
            SolveHungrian(costMatrix, N, M, assignment);
        }
        else
        {
            SolveBipartiteGraphs(costMatrix, N, M, assignment, maxCost);
        }

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
/// \brief CTracker::CreateDistaceMatrix
/// \param regions
/// \param costMatrix
/// \param maxPossibleCost
/// \param maxCost
///
void CTracker::CreateDistaceMatrix(const regions_t& regions, distMatrix_t& costMatrix, track_t maxPossibleCost, track_t& maxCost, cv::UMat currFrame)
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
/// \brief CTracker::SolveHungrian
/// \param costMatrix
/// \param N
/// \param M
/// \param assignment
///
void CTracker::SolveHungrian(const distMatrix_t& costMatrix, size_t N, size_t M, assignments_t& assignment)
{
    AssignmentProblemSolver APS;
    APS.Solve(costMatrix, N, M, assignment, AssignmentProblemSolver::optimal);
}

///
/// \brief CTracker::SolveBipartiteGraphs
/// \param costMatrix
/// \param N
/// \param M
/// \param assignment
/// \param maxCost
///
void CTracker::SolveBipartiteGraphs(const distMatrix_t& costMatrix, size_t N, size_t M, assignments_t& assignment, track_t maxCost)
{
    MyGraph G;
    G.make_directed();

    std::vector<node> nodes(N + M);

    for (size_t i = 0; i < nodes.size(); ++i)
    {
        nodes[i] = G.new_node();
    }

    edge_map<int> weights(G, 100);
    for (size_t i = 0; i < N; i++)
    {
        bool hasZeroEdge = false;

        for (size_t j = 0; j < M; j++)
        {
            track_t currCost = costMatrix[i + j * N];

            edge e = G.new_edge(nodes[i], nodes[N + j]);

            if (currCost < m_settings.m_distThres)
            {
                int weight = static_cast<int>(maxCost - currCost + 1);
                G.set_edge_weight(e, weight);
                weights[e] = weight;
            }
            else
            {
                if (!hasZeroEdge)
                {
                    G.set_edge_weight(e, 0);
                    weights[e] = 0;
                }
                hasZeroEdge = true;
            }
        }
    }

    edges_t L = MAX_WEIGHT_BIPARTITE_MATCHING(G, weights);
    for (edges_t::iterator it = L.begin(); it != L.end(); ++it)
    {
        node a = it->source();
        node b = it->target();
        assignment[b.id()] = static_cast<assignments_t::value_type>(a.id() - N);
    }
}
