#ifndef UTILS_H
#define UTILS_H

#include "core/parameters.h"
#include "core/datatypes/vehicle_state.h"
#include "core/datatypes/ego_state.h"
#include "core/datatypes/trajectory.h"

double ComputeLaneCenterYCoordinate(const int lane_id, const Parameters& parameters);

double ComputeEgoDistanceIncrement(const Parameters& parameters, const EgoState& ego_state);

Trajectory ComputeStraightTrajectory(const double x_start, const double y, const double x_increment, const int size);

#endif // UTILS_H
