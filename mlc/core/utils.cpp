#include "core/utils.h"

double ComputeLaneCenterYCoordinate(const int lane_id, const Parameters& parameters)
{
    return (parameters.number_of_lanes - lane_id) * parameters.lane_width - (parameters.lane_width/2);
}