#include "collision_detector.h"
#include "utils.h"
#include <algorithm>
#include<math.h>

int ComputeNumberOfNeededTrajectorySamples(const Parameters& parameters)
{
    return static_cast<int>(parameters.collision_horizon_time / parameters.cycle_time);
}

Trajectory ExtendLaneChangeTrajectory(const Trajectory& lane_change_trajectory,
                                      const int desired_number_of_samples,
                                      const Parameters& parameters)
{
    Trajectory extended_trajectory{lane_change_trajectory};

    const int number_of_samples_to_add{desired_number_of_samples - static_cast<int>(lane_change_trajectory.size())};

    if (number_of_samples_to_add > 0)
    {
        const auto x_increment{ComputeEgoDistanceIncrement(parameters)};
        const auto x_start{lane_change_trajectory.back().x + x_increment};
        const auto y{lane_change_trajectory.back().y};
        const auto extention{ComputeStraightTrajectory(x_start, y, x_increment, number_of_samples_to_add)};
        extended_trajectory.insert(extended_trajectory.end(), extention.begin(), extention.end());
    }

    return extended_trajectory;
}

int FindCollidingObjectIndex(const VehicleState& ego_state, const std::vector<VehicleState> objects)
{
    const auto it{std::find_if(std::begin(objects), std::end(objects), [ego_state](const auto& object){
        return ((ego_state.lane_id + 1) == object.lane_id);
    })};

    return (it - std::begin(objects));
}

Trajectory ComputeObjectTrajectory(const VehicleState& vehicle_state,
                                   const int desired_number_of_samples,
                                   const Parameters& parameters)
{
    Trajectory trajectory{};

    const auto x_increment{vehicle_state.speed * 1000.0 / 3600 * parameters.cycle_time};
    const auto x_start{vehicle_state.x_coordinate + x_increment};
    const auto y{ComputeLaneCenterYCoordinate(vehicle_state.lane_id, parameters)};

    return ComputeStraightTrajectory(x_start, y, x_increment, desired_number_of_samples);
}



double CalculateDistance(TrajectoryPoint p1, TrajectoryPoint p2)
{
	double x{p1.x - p2.x};
	double y{p1.y - p2.y};
	double squared_distance{std::pow(x, 2) + std::pow(y, 2)};

	return sqrt(squared_distance);
}

bool DoTrajectoriesCollide(const Trajectory& lhs, const Trajectory& rhs, const double collision_distance)
{
    const auto num_vertices{std::min(lhs.size(), rhs.size())};

    for (int i{0}; i < num_vertices; ++i)
    {
        const auto vertex1{lhs.at(i)};
        const auto vertex2{rhs.at(i)};
        const auto distance{CalculateDistance(vertex1, vertex2)};

        if (distance < collision_distance)
        {
            return true;
        }
    }

    return false;
}

bool EgoCollidesWithObject(const VehicleState& ego_state,
                           const Trajectory& ego_lane_change_trajectory,
                           const std::vector<VehicleState> objects,
                           const Parameters& parameters)
{
    const auto desired_number_of_samples{ComputeNumberOfNeededTrajectorySamples(parameters)};
    const auto extended_ego_lane_trajectory{ExtendLaneChangeTrajectory(ego_lane_change_trajectory,
                                                                       desired_number_of_samples,
                                                                       parameters)};
    const auto object_index{FindCollidingObjectIndex(ego_state, objects)};


    if (object_index >= objects.size())
    {
        return false;
    }

    const auto object{objects.at(object_index)};
    const auto object_trajectory{ComputeObjectTrajectory(object, desired_number_of_samples, parameters)};

    return DoTrajectoriesCollide(extended_ego_lane_trajectory, object_trajectory, parameters.collision_distance);
}
