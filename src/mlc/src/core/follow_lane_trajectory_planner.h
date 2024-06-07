#ifndef FOLLOW_LANE_TRAJECTORY_PLANNER_H
#define FOLLOW_LANE_TRAJECTORY_PLANNER_H

#include "parameters.h"
#include "datatypes/trajectory.h"
#include "datatypes/ego_state.h"

Trajectory ComputeFollowLaneTrajectory(const EgoState& ego_state, const Parameters& parameters);

#endif  // FOLLOW_LANE_TRAJECTORY_PLANNER_H
