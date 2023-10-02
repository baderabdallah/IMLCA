#include "core/follow_lane_trajectory_planner.h"
#include "core/utils.h"

Trajectory ComputeFollowLaneTrajectory(const VehicleState& vehicle_state, const Parameters& parameters)
{
    Trajectory trajectory{};

    const auto x_increment{parameters.ego_speed * 1000.0 / 3600.0 * parameters.cycle_time};
    const auto number_of_vertices{static_cast<int>(parameters.follow_lane_trajectory_length / x_increment)};

    for (int i{1}; i < number_of_vertices; ++i)
    {
        TrajectoryPoint point{};
        point.x = vehicle_state.x_coordinate + i * x_increment;
        point.y = ComputeLaneCenterYCoordinate(vehicle_state.lane_id, parameters);

        trajectory.push_back(point);
    }

    return trajectory;
}
