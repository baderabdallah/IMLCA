#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "parameters.h"
#include "datatypes/trajectory.h"
#include "datatypes/vehicle_state.h"

bool EgoCollidesWithObject(const VehicleState& ego_state,
                           const Trajectory& ego_lane_change_trajectory,
                           const std::vector<VehicleState> objects,
                           const Parameters& parameters);

#endif  // COLLISION_DETECTOR_H
