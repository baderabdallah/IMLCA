#ifndef MULTIPLE_LANE_CHANGE_H
#define MULTIPLE_LANE_CHANGE_H

#include "core/datatypes/trajectory.h"
#include "core/datatypes/vehicle_state.h"
#include "core/finite_state_machine.h"
#include "core/parameters.h"
#include <vector>

class MultipleLaneChange
{
  public:
    MultipleLaneChange(const Parameters& parameters);

    void SetObjects(const std::vector<VehicleState>& objects);
    void Step();
    Trajectory GetEgoTrajectory() const;

  private:
    enum class MotionStates
    {
        kFollowLane,
        kChangeLane
    };

    enum class MotionTransitions
    {
        kStartLaneChange,
        kLaneChangeCompleted
    };

    using MotionStateMachine = FiniteStateMachine<MotionStates, MotionTransitions>;

    void HandleFollowLaneState();
    bool EgoReachedTargetLane() const;
    void KeepFollowingLane();
    void StartLaneChange(const Trajectory& lane_change_trajectory);
    void UpdateEgoState(const Trajectory& ego_trajectory);
    void HandleLaneChangeState();
    void ConsumeLaneChangeTrajectory();
    bool LaneChangeTrajectoryFullyConsumed() const;
    void StartFollowingLane();

    const Parameters parameters_{};
    std::vector<VehicleState> objects_{};
    Trajectory ego_trajectory_{};
    MotionStateMachine state_machine_{MotionStates::kFollowLane};
    VehicleState ego_state_{};
};

#endif // MULTIPLE_LANE_CHANGE_H
