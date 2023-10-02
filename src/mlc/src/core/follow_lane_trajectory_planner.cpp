#include "core/follow_lane_trajectory_planner.h"
#include "core/utils.h"

Trajectory ComputeFollowLaneTrajectory(const VehicleState& vehicle_state, const Parameters& parameters)
{
    const auto x_increment{ComputeEgoDistanceIncrement(parameters)};
    const auto size{static_cast<int>(parameters.follow_lane_trajectory_length / x_increment)};
    const auto x_start{vehicle_state.x_coordinate + x_increment};
    const auto y{ComputeLaneCenterYCoordinate(vehicle_state.lane_id, parameters)};

    return ComputeStraightTrajectory(x_start, y, x_increment, size);
}
