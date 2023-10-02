#ifndef PARAMETERS_H
#define PARAMETERS_H

struct Parameters
{
    // Lane properties
    double lane_width{4.0};  // m
    int number_of_lanes{5};

    // System properties
    double cycle_time{0.1};  // s

    // Ego behavior
    double ego_speed{90.0};  // km/h
    double lane_change_longitudinal_distance{10.0};  // m
    double collision_horizon_time{10.0};  // s
    double follow_lane_trajectory_length{50.0};  // m
};

#endif // PARAMETERS_H
