#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "core/parameters.h"
#include "core/datatypes/trajectory.h"
#include "core/datatypes/vehicle_state.h"

bool EgoCollidesWithObject(const VehicleState& ego_state,
                           const Trajectory& ego_lane_change_trajectory,
                           const std::vector<VehicleState> objects);

#endif  // COLLISION_DETECTOR_H
