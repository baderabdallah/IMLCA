#include "core/utils.h"
#include <iostream>

double ComputeLaneCenterYCoordinate(const int lane_id, const Parameters& parameters)
{
    return (parameters.number_of_lanes - lane_id) * parameters.lane_width - (parameters.lane_width/2);
}

double ComputeEgoDistanceIncrement(const Parameters& parameters, const EgoState& ego_state)
{
    std::cout << "Speed:" << ego_state.speed << std::endl;
    return (ego_state.speed * 1000.0 / 3600.0 * parameters.cycle_time);
}

Trajectory ComputeStraightTrajectory(const double x_start, const double y, const double x_increment, const int size)
{
    Trajectory trajectory{};

    for (int i{0}; i < size; ++i)
    {
        TrajectoryPoint point{};
        point.x = x_start + i * x_increment;
        point.y = y;

        trajectory.push_back(point);
    }

    return trajectory;
}