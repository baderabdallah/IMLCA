#ifndef LANE_CHANGE_TRAJECTORY_PLANNER_H
#define LANE_CHANGE_TRAJECTORY_PLANNER_H

#include "core/parameters.h"
#include "core/datatypes/trajectory.h"
#include "core/datatypes/vehicle_state.h"

Trajectory ComputeLaneChangeTrajectory(const VehicleState& ego_vehicle_state, const Parameters& parameters);
Trajectory ComputeFollowLaneTrajectory(const Parameters& parameters);

#endif  // LANE_CHANGE_TRAJECTORY_PLANNER_H
