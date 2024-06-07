#include "core/follow_lane_trajectory_planner.h"
#include "core/utils.h"
#include <iostream>

Trajectory ComputeFollowLaneTrajectory(const EgoState& ego_state, const Parameters& parameters)
{
    std::cout << "ComputeFollowLaneTrajectory" << std::endl;

    const auto x_increment{ComputeEgoDistanceIncrement(parameters, ego_state)};
    std::cout<< "increment:" << x_increment << std::endl;
    const auto size{static_cast<int>(parameters.follow_lane_trajectory_length / x_increment)};
    
    const auto x_start{ego_state.x_coordinate + x_increment};
    
    const auto y{ComputeLaneCenterYCoordinate(ego_state.lane_id, parameters)};

    return ComputeStraightTrajectory(x_start, y, x_increment, size);
}
