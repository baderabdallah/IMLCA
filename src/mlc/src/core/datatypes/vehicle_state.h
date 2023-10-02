#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

struct VehicleState
{
    int lane_id{0};
    double x_coordinate{0.0};
    double speed{0.0};  // km/h
};

#endif  // VEHICLE_STATE_H
