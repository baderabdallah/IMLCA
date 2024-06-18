#ifndef LANE_CHANGE_TRAJECTORY_PLANNER_H
#define LANE_CHANGE_TRAJECTORY_PLANNER_H

#include "core/parameters.h"
#include "core/datatypes/trajectory.h"
#include "core/datatypes/ego_state.h"

enum class LaneChangeCommandDirection
{
    kLeft,
    kRight,
};

Trajectory ComputeLaneChangeTrajectory(const EgoState& ego_state, const Parameters& parameters, LaneChangeCommandDirection lane_change_direction);
// Trajectory ComputeFollowLaneTrajectory(const EgoState& ego_state, const Parameters& parameters);

#endif  // LANE_CHANGE_TRAJECTORY_PLANNER_H
