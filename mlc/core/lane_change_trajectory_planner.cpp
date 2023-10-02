#include "core/lane_change_trajectory_planner.h"
#include "core/utils.h"
#include <cmath>
#include <iostream>
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

Trajectory ComputeLaneChangeTrajectory(const VehicleState& ego_vehicle_state, const Parameters& parameters)
{
    float speed_m_s = parameters.ego_speed * 1000 /(60*60);

    int horizon_m = 1000; //m

    double delta_time_sec_y = parameters.lane_change_longitudinal_distance / speed_m_s;

    int delta_time_sec_x = horizon_m/speed_m_s;
    int number_of_steps = std::floor(delta_time_sec_y/parameters.cycle_time);
    std::vector<int> time_steps_y(number_of_steps);

    for(size_t i = 0; i < time_steps_y.size(); i++)
    {
        time_steps_y.at(i) = parameters.cycle_time * i;
    }

    Trajectory computedTrajectory(number_of_steps);

    float coeffA = 0.5;
    float coeffB =  2.35;

    float coeffC = static_cast<float>(ComputeLaneCenterYCoordinate(ego_vehicle_state.lane_id, parameters));

    for (size_t j = 0; j < computedTrajectory.size(); j++)
    {
        computedTrajectory.at(j).y =  - coeffA * (time_steps_y.at(j) * time_steps_y.at(j)) - coeffB * time_steps_y.at(j) + coeffC;
    }

    int horizon_steps = delta_time_sec_x/parameters.cycle_time;
    float distance_increment_in_x_coordinates = float(horizon_m) / horizon_steps;

    for (size_t q = 0; q < computedTrajectory.size(); q++)
    {
        computedTrajectory.at(q).x = (ego_vehicle_state.x_coordinate) + (distance_increment_in_x_coordinates) * q;
    }

    return computedTrajectory;

}
