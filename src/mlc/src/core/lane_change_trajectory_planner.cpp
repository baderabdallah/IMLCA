#include "core/lane_change_trajectory_planner.h"
#include "core/utils.h"
#include <cmath>
// #include <iostream>
Trajectory ComputeFollowLaneTrajectory(const Parameters& parameters)
{
    float speed_m_s = parameters.ego_speed * 1000 /(60*60);

    int horizon_m = 1000; //m

    int delta_time_sec = horizon_m/speed_m_s;

    int horizon_steps = delta_time_sec/parameters.cycle_time;

    float distance_increment_in_x_coordinates = horizon_m / horizon_steps;

    Trajectory computedTrajectory(horizon_steps);
    for (int q = 0; q < (int) computedTrajectory.size(); q++)
    {
     computedTrajectory.at(q).x = (distance_increment_in_x_coordinates) * q;
    }

    return computedTrajectory;
}

Trajectory ComputeLaneChangeTrajectory(const VehicleState& ego_vehicle_state, const Parameters& parameters, bool is_lane_change_right)
{
    float speed_m_s = parameters.ego_speed * 1000 /(60*60);
    double delta_time_sec_y = parameters.lane_change_longitudinal_distance / speed_m_s;

    auto y_start = ComputeLaneCenterYCoordinate(ego_vehicle_state.lane_id, parameters);
    int number_of_steps = std::floor(delta_time_sec_y/parameters.cycle_time);
    
    auto y_end = y_start;
    if (is_lane_change_right){
        // right
        y_end = y_end - parameters.lane_width;
    }
    else {
        // left
        y_end = y_end + parameters.lane_width;
    }

    auto x_start = ego_vehicle_state.x_coordinate;
    auto x_end = x_start + parameters.lane_change_longitudinal_distance;

    Trajectory computedTrajectory{};

    auto distance_increment_x = (x_end - x_start) / number_of_steps;
    auto distance_increment_y = (y_end - y_start) / number_of_steps;

    for(int i = 0; i<=number_of_steps; i++)
    {
        TrajectoryPoint trajectory_point{};
        trajectory_point.x = x_start + i * distance_increment_x;
        trajectory_point.y = y_start + i * distance_increment_y;
        computedTrajectory.push_back(trajectory_point);
    }

    return computedTrajectory;

}
