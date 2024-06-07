#ifndef MULTIPLE_LANE_CHANGE_H
#define MULTIPLE_LANE_CHANGE_H

#include "core/datatypes/trajectory.h"
#include "core/datatypes/vehicle_state.h"
#include "core/datatypes/ego_state.h"
#include "core/finite_state_machine.h"
#include "core/parameters.h"
#include <vector>
#include <string>

class MultipleLaneChange
{
  public:
    MultipleLaneChange(const Parameters& parameters, const EgoState& ego_state);

    void SetObjects(const std::vector<VehicleState>& objects);
    void Step();
    void SetKeyboardInput(const std::string);

    Trajectory GetEgoTrajectory() const;
    EgoState GetEgoState() const;

  private:
    enum class MotionStates
    {
        kFollowLane,
        kChangeLaneLeft,
        kChangeLaneRight
    };

    enum class MotionTransitions
    {
        kStartLaneChangeRight,
        kStartLaneChangeLeft,
        kLaneChangeRightCompleted,
        kLaneChangeLeftCompleted
    };

    using MotionStateMachine = FiniteStateMachine<MotionStates, MotionTransitions>;

    void HandleFollowLaneState();
    void KeepFollowingLane();
    void StartLaneChange();
    void UpdateEgoState(const Trajectory& ego_trajectory);
    void HandleLaneChangeState(MultipleLaneChange::MotionStates motion_state);
    void ConsumeLaneChangeTrajectory();
    bool LaneChangeTrajectoryFullyConsumed() const;
    void StartFollowingLane(MultipleLaneChange::MotionStates motion_state);

    const Parameters parameters_{};
    EgoState ego_state_{};
    std::vector<VehicleState> objects_{};
    Trajectory ego_trajectory_{};
    MotionStateMachine state_machine_{MotionStates::kFollowLane};
    std::string kb_input_ = "";
};

#endif // MULTIPLE_LANE_CHANGE_H
