#ifndef EGO_STATE_H
#define EGO_STATE_H

struct EgoState
{
    int lane_id{0};
    double x_coordinate{0.0};
    double speed{0.0};  // km/h
};

#endif  // EGO_STATE_H
