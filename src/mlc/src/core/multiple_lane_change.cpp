#include "core/multiple_lane_change.h"
#include "core/follow_lane_trajectory_planner.h"
#include "core/lane_change_trajectory_planner.h"
#include <algorithm> // For std::copy

MultipleLaneChange::MultipleLaneChange(const Parameters &parameters, const EgoState &ego_state)
    : parameters_{parameters}, ego_state_{ego_state}
{
    // Define the state machine transitions
    state_machine_.AddTransition(MotionStates::kFollowLane, MotionTransitions::kStartLaneChangeRight, MotionStates::kChangeLaneRight, [](){});
    state_machine_.AddTransition(MotionStates::kChangeLaneRight, MotionTransitions::kLaneChangeRightCompleted, MotionStates::kFollowLane, [](){});
    state_machine_.AddTransition(MotionStates::kFollowLane, MotionTransitions::kStartLaneChangeLeft, MotionStates::kChangeLaneLeft, [](){});
    state_machine_.AddTransition(MotionStates::kChangeLaneLeft, MotionTransitions::kLaneChangeLeftCompleted, MotionStates::kFollowLane, [](){});
}

void MultipleLaneChange::SetObjects(const std::vector<VehicleState> &objects)
{
    objects_ = objects;
}

void MultipleLaneChange::SetKeyboardInput(const std::string& kb_input)
{
    kb_input_ = kb_input;
}

void MultipleLaneChange::Step()
{
    HandleSpeedInput();

    if (MotionStates::kFollowLane == state_machine_.GetCurrentState())
    {
        HandleFollowLaneState();
    }
    else
    {
        HandleLaneChangeState(state_machine_.GetCurrentState());
    }

    // Reset keyboard input after step is done to process it only once
    kb_input_.clear();
}

Trajectory MultipleLaneChange::GetEgoTrajectory() const
{
    // Convert deque to vector for the return type
    Trajectory trajectory_vec;
    trajectory_vec.reserve(ego_trajectory_.size());
    std::copy(ego_trajectory_.begin(), ego_trajectory_.end(), std::back_inserter(trajectory_vec));
    return trajectory_vec;
}

EgoState MultipleLaneChange::GetEgoState() const
{
    return ego_state_;
}

bool MultipleLaneChange::CanChangeLane() const
{
    return following_lane_counter_ > kLaneChangeDebounce;
}

void MultipleLaneChange::HandleSpeedInput()
{
    if (kb_input_ == KeyboardCommands::LEFT_ARROW)
    {
        ego_state_.speed = std::max(0.0, ego_state_.speed - kSpeedIncrement);
    }
    else if (kb_input_ == KeyboardCommands::RIGHT_ARROW)
    {
        ego_state_.speed = std::min(120.0, ego_state_.speed + kSpeedIncrement);
    }
}

void MultipleLaneChange::HandleFollowLaneState()
{
    if (CanChangeLane() && (kb_input_ == KeyboardCommands::UP_ARROW || kb_input_ == KeyboardCommands::DOWN_ARROW))
    {
        StartLaneChange();
    }
    else
    {
        KeepFollowingLane();
    }
}

void MultipleLaneChange::KeepFollowingLane()
{
    const auto trajectory = ComputeFollowLaneTrajectory(ego_state_, parameters_);
    ego_trajectory_.assign(trajectory.begin(), trajectory.end());
    UpdateEgoState(trajectory);
    following_lane_counter_++;
}

void MultipleLaneChange::StartLaneChange()
{
    const bool is_not_in_leftmost_lane = ego_state_.lane_id > 0;
    const bool is_not_in_rightmost_lane = ego_state_.lane_id < parameters_.number_of_lanes - 1;

    Trajectory lane_change_trajectory;

    if (kb_input_ == KeyboardCommands::UP_ARROW && is_not_in_leftmost_lane)
    {
        lane_change_trajectory = ComputeLaneChangeTrajectory(ego_state_, parameters_, LaneChangeCommandDirection::kLeft);
        state_machine_.HandleEvent(MotionTransitions::kStartLaneChangeLeft);
    }
    else if (kb_input_ == KeyboardCommands::DOWN_ARROW && is_not_in_rightmost_lane)
    {
        lane_change_trajectory = ComputeLaneChangeTrajectory(ego_state_, parameters_, LaneChangeCommandDirection::kRight);
        state_machine_.HandleEvent(MotionTransitions::kStartLaneChangeRight);
    }
    else
    {
        // Invalid lane change command for the current lane
        KeepFollowingLane();
        return;
    }
    
    ego_trajectory_.assign(lane_change_trajectory.begin(), lane_change_trajectory.end());
    UpdateEgoState(lane_change_trajectory);
    following_lane_counter_ = 0;
}

void MultipleLaneChange::UpdateEgoState(const Trajectory &ego_trajectory)
{
    if (!ego_trajectory.empty())
    {
        ego_state_.x_coordinate = ego_trajectory.front().x;
    }
}

void MultipleLaneChange::HandleLaneChangeState(MotionStates motion_state)
{
    ConsumeLaneChangeTrajectory();

    if (LaneChangeTrajectoryFullyConsumed())
    {
        StartFollowingLane(motion_state);
    }
}

void MultipleLaneChange::ConsumeLaneChangeTrajectory()
{
    if (!ego_trajectory_.empty())
    {
        ego_trajectory_.pop_front();
        if (!ego_trajectory_.empty())
        {
            ego_state_.x_coordinate = ego_trajectory_.front().x;
        }
    }
}

bool MultipleLaneChange::LaneChangeTrajectoryFullyConsumed() const
{
    // The trajectory is considered consumed when only the last point remains
    return ego_trajectory_.size() <= 1;
}

void MultipleLaneChange::StartFollowingLane(MotionStates motion_state)
{
    if (motion_state == MotionStates::kChangeLaneRight)
    {
        ++ego_state_.lane_id;
        state_machine_.HandleEvent(MotionTransitions::kLaneChangeRightCompleted);
    }
    else if (motion_state == MotionStates::kChangeLaneLeft)
    {
        --ego_state_.lane_id;
        state_machine_.HandleEvent(MotionTransitions::kLaneChangeLeftCompleted);
    }
    KeepFollowingLane(); // Immediately generate a follow-lane trajectory
}