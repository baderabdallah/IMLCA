#ifndef MULTIPLE_LANE_CHANGE_H
#define MULTIPLE_LANE_CHANGE_H

#include "core/datatypes/trajectory.h"
#include "core/datatypes/vehicle_state.h"
#include "core/datatypes/ego_state.h"
#include "core/finite_state_machine.h"
#include "core/parameters.h"
#include <vector>
#include <string>
#include <deque>

// Constants for keyboard inputs to avoid magic strings
namespace KeyboardCommands {
    const std::string LEFT_ARROW = "Left Arrow";
    const std::string RIGHT_ARROW = "Right Arrow";
    const std::string UP_ARROW = "Up Arrow";
    const std::string DOWN_ARROW = "Down Arrow";
}

class MultipleLaneChange
{
  public:
    MultipleLaneChange(const Parameters& parameters, const EgoState& ego_state);

    void SetObjects(const std::vector<VehicleState>& objects);
    void Step();
    void SetKeyboardInput(const std::string& kb_input);

    Trajectory GetEgoTrajectory() const;
    EgoState GetEgoState() const;
    bool CanChangeLane() const;

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

    // State handlers
    void HandleFollowLaneState();
    void HandleLaneChangeState(MotionStates motion_state);

    // Behavior logic
    void KeepFollowingLane();
    void StartLaneChange();
    void ConsumeLaneChangeTrajectory();
    void StartFollowingLane(MotionStates motion_state);
    void HandleSpeedInput();

    // Helpers
    void UpdateEgoState(const Trajectory& ego_trajectory);
    bool LaneChangeTrajectoryFullyConsumed() const;

    const Parameters parameters_{};
    EgoState ego_state_{};
    std::vector<VehicleState> objects_{};
    std::deque<TrajectoryPoint> ego_trajectory_{}; // Use deque for efficient front removal
    MotionStateMachine state_machine_{MotionStates::kFollowLane};
    std::string kb_input_ = "";
    
    // Counter to prevent immediate lane changes after completing one
    int following_lane_counter_{0}; 
    static constexpr int kLaneChangeDebounce = 3; // Number of steps to wait before allowing another lane change
    static constexpr double kSpeedIncrement = 2.0; // Speed change in kph
};

#endif // MULTIPLE_LANE_CHANGE_H
