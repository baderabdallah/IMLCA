#include "follow_lane_trajectory_planner.h"
#include "lane_change_trajectory_planner.h"
#include "multiple_lane_change.h"
#include <iostream>

MultipleLaneChange::MultipleLaneChange(const Parameters& parameters, const EgoState& ego_state)
  : parameters_{parameters}, ego_state_{ego_state}
{
    std::cout << "Create MultipleLaneChange" <<std::endl;
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
    std::cout << "SetObjects" <<std::endl;
    objects_ = objects;
}

void MultipleLaneChange::SetKeyboardInput(const std::string kb_input){
    std::cout << "SetKeyboardInput" <<std::endl;
    kb_input_ = kb_input;
}

void MultipleLaneChange::Step()
{
    std::cout << "Step" <<std::endl;
    std::cout << "ego_state_.speed " << ego_state_.speed <<std::endl;

    if (kb_input_ == "Left Arrow"){
        ego_state_.speed--;
    }
    if (kb_input_ == "Right Arrow"){
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
    std::cout << "GetEgoTrajectory" <<std::endl;
    return ego_trajectory_;
}

EgoState MultipleLaneChange::GetEgoState() const
{
    std::cout << "GetEgoState" <<std::endl;
    return ego_state_;
}

void MultipleLaneChange::HandleFollowLaneState()
{
    std::cout << "HandleFollowLaneState" <<std::endl;
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
    std::cout << "KeepFollowingLane" <<std::endl;
    ego_trajectory_ = ComputeFollowLaneTrajectory(ego_state_, parameters_);
    UpdateEgoState(ego_trajectory_);
}

void MultipleLaneChange::StartLaneChange()
{
    std::cout << "StartLaneChange" <<std::endl;
    bool is_ego_in_left_most_lane = ego_state_.lane_id > 0;
    bool is_ego_in_right_most_lane = ego_state_.lane_id < parameters_.number_of_lanes - 1;

    if (kb_input_ == "Up Arrow" && is_ego_in_left_most_lane) {
        const auto lane_change_trajectory{ComputeLaneChangeTrajectory(ego_state_, parameters_, LaneChangeCommandDirection::kLeft)};
        ego_trajectory_ = lane_change_trajectory;
        UpdateEgoState(ego_trajectory_);
        state_machine_.HandleEvent(MotionTransitions::kStartLaneChangeLeft);
    }
    else if (kb_input_ == "Down Arrow" && is_ego_in_right_most_lane) {
        const auto lane_change_trajectory{ComputeLaneChangeTrajectory(ego_state_, parameters_, LaneChangeCommandDirection::kRight)};
        ego_trajectory_ = lane_change_trajectory;
        UpdateEgoState(ego_trajectory_);
        state_machine_.HandleEvent(MotionTransitions::kStartLaneChangeRight);
    }
}

void MultipleLaneChange::UpdateEgoState(const Trajectory& ego_trajectory)
{
    std::cout << "UpdateEgoState" <<std::endl;
    ego_state_.x_coordinate = ego_trajectory.at(0).x;
}

void MultipleLaneChange::HandleLaneChangeState(MultipleLaneChange::MotionStates motion_state)
{
    std::cout << "HandleLaneChangeState" <<std::endl;
    ConsumeLaneChangeTrajectory();

    if (LaneChangeTrajectoryFullyConsumed())
    {
        StartFollowingLane(motion_state);
    }
}

void MultipleLaneChange::ConsumeLaneChangeTrajectory()
{
    std::cout << "ConsumeLaneChangeTrajectory" <<std::endl;
    ego_trajectory_.erase(ego_trajectory_.begin());
    UpdateEgoState(ego_trajectory_);
}

bool MultipleLaneChange::LaneChangeTrajectoryFullyConsumed() const
{
    std::cout << "LaneChangeTrajectoryFullyConsumed" <<std::endl;
    return (1 == ego_trajectory_.size());
}

void MultipleLaneChange::StartFollowingLane(MultipleLaneChange::MotionStates motion_state)
{
    std::cout << "StartFollowingLane" <<std::endl;
    if (motion_state == MotionStates::kChangeLaneRight) {
        ++ego_state_.lane_id;
        state_machine_.HandleEvent(MotionTransitions::kLaneChangeRightCompleted);
    }
    else if (motion_state == MotionStates::kChangeLaneLeft) {
        --ego_state_.lane_id;
        state_machine_.HandleEvent(MotionTransitions::kLaneChangeLeftCompleted);
    }
}
