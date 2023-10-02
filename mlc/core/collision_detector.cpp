#include "core/collision_detector.h"

bool EgoCollidesWithObject(const VehicleState& ego_state,
                           const Trajectory& ego_lane_change_trajectory,
                           const std::vector<VehicleState> objects)
{
    (void) ego_state;
    (void) ego_lane_change_trajectory;
    (void) objects;

    return true;
}