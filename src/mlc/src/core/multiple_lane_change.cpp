#include "follow_lane_trajectory_planner.h"
#include "lane_change_trajectory_planner.h"
#include "multiple_lane_change.h"

MultipleLaneChange::MultipleLaneChange(const Parameters& parameters, const EgoState& ego_state)
  : parameters_{parameters}, ego_state_{ego_state}
{
    state_machine_.AddTransition(MotionStates::kFollowLane,
                                 MotionTransitions::kStartLaneChangeRight,
                                 MotionStates::kChangeLaneRight,
                                 [](){});
    state_machine_.AddTransition(MotionStates::kChangeLaneRight,
                                 MotionTransitions::kLaneChangeRightCompleted,
                                 MotionStates::kFollowLane,
                                 [](){});
    state_machine_.AddTransition(MotionStates::kFollowLane,
                                 MotionTransitions::kStartLaneChangeLeft,
                                 MotionStates::kChangeLaneLeft,
                                 [](){});
    state_machine_.AddTransition(MotionStates::kChangeLaneLeft,
                                 MotionTransitions::kLaneChangeLeftCompleted,
                                 MotionStates::kFollowLane,
                                 [](){});
}

void MultipleLaneChange::SetObjects(const std::vector<VehicleState>& objects)
{
    objects_ = objects;
}

void MultipleLaneChange::SetKeyboardInput(const std::string kb_input){
    kb_input_ = kb_input;
}

void MultipleLaneChange::Step()
{
    if (kb_input_ == "Left Arrow" && ego_state_.speed > 0)
    {
        ego_state_.speed--;
        ego_state_.speed--;
    }
    if (kb_input_ == "Right Arrow" && ego_state_.speed < 120){
        ego_state_.speed++;
        ego_state_.speed++;
    }

    if (MotionStates::kFollowLane == state_machine_.GetCurrentState())
        {
            HandleFollowLaneState();
        }
    else
        {
            HandleLaneChangeState(state_machine_.GetCurrentState());
        }

    // reset keyboard input after step is done
    SetKeyboardInput("");
}

Trajectory MultipleLaneChange::GetEgoTrajectory() const
{
    return ego_trajectory_;
}

EgoState MultipleLaneChange::GetEgoState() const
{
    return ego_state_;
}

bool MultipleLaneChange::isActionPossible() const
{
    return following_lane_counter > 2;
}

void MultipleLaneChange::HandleFollowLaneState()
{
    if (kb_input_ != "Up Arrow" && kb_input_ != "Down Arrow") {
        KeepFollowingLane();
    }
    else
    {
        StartLaneChange();
    }
}

void MultipleLaneChange::KeepFollowingLane()
{
    ego_trajectory_ = ComputeFollowLaneTrajectory(ego_state_, parameters_);
    UpdateEgoState(ego_trajectory_);
    following_lane_counter += 1;
}

void MultipleLaneChange::StartLaneChange()
{
    bool is_ego_not_in_left_most_lane = ego_state_.lane_id > 0;
    bool is_ego_not_in_right_most_lane = ego_state_.lane_id < parameters_.number_of_lanes - 1;

    if (kb_input_ == "Up Arrow" && is_ego_not_in_left_most_lane) {
        const auto lane_change_trajectory{ComputeLaneChangeTrajectory(ego_state_, parameters_, LaneChangeCommandDirection::kLeft)};
        ego_trajectory_ = lane_change_trajectory;
        UpdateEgoState(ego_trajectory_);
        state_machine_.HandleEvent(MotionTransitions::kStartLaneChangeLeft);
        following_lane_counter = 0;
    }
    else if (kb_input_ == "Down Arrow" && is_ego_not_in_right_most_lane) {
        const auto lane_change_trajectory{ComputeLaneChangeTrajectory(ego_state_, parameters_, LaneChangeCommandDirection::kRight)};
        ego_trajectory_ = lane_change_trajectory;
        UpdateEgoState(ego_trajectory_);
        state_machine_.HandleEvent(MotionTransitions::kStartLaneChangeRight);
        following_lane_counter = 0;
    } else {
        KeepFollowingLane();
    }
}

void MultipleLaneChange::UpdateEgoState(const Trajectory& ego_trajectory)
{
    ego_state_.x_coordinate = ego_trajectory.at(0).x;
}

void MultipleLaneChange::HandleLaneChangeState(MultipleLaneChange::MotionStates motion_state)
{
    ConsumeLaneChangeTrajectory();

    if (LaneChangeTrajectoryFullyConsumed())
    {
        StartFollowingLane(motion_state);
    }
}

void MultipleLaneChange::ConsumeLaneChangeTrajectory()
{
    ego_trajectory_.erase(ego_trajectory_.begin());
    UpdateEgoState(ego_trajectory_);
}

bool MultipleLaneChange::LaneChangeTrajectoryFullyConsumed() const
{
    return (1 == ego_trajectory_.size());
}

void MultipleLaneChange::StartFollowingLane(MultipleLaneChange::MotionStates motion_state)
{
    if (motion_state == MotionStates::kChangeLaneRight) {
        ++ego_state_.lane_id;
        state_machine_.HandleEvent(MotionTransitions::kLaneChangeRightCompleted);
    }
    else if (motion_state == MotionStates::kChangeLaneLeft) {
        --ego_state_.lane_id;
        state_machine_.HandleEvent(MotionTransitions::kLaneChangeLeftCompleted);
    }
}
