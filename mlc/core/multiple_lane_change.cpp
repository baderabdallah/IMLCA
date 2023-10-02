#include "collision_detector.h"
#include "follow_lane_trajectory_planner.h"
#include "lane_change_trajectory_planner.h"
#include "multiple_lane_change.h"

MultipleLaneChange::MultipleLaneChange(const Parameters& parameters)
  : parameters_{parameters}
{
    state_machine_.AddTransition(MotionStates::kFollowLane,
                                 MotionTransitions::kStartLaneChange,
                                 MotionStates::kChangeLane,
                                 [](){});
    state_machine_.AddTransition(MotionStates::kChangeLane,
                                 MotionTransitions::kLaneChangeCompleted,
                                 MotionStates::kFollowLane,
                                 [](){});
}

void MultipleLaneChange::SetObjects(const std::vector<VehicleState>& objects)
{
    objects_ = objects;
}

void MultipleLaneChange::Step()
{
    if (MotionStates::kFollowLane == state_machine_.GetCurrentState())
    {
        HandleFollowLaneState();
    }
    else
    {
        HandleLaneChangeState();
    }
}

Trajectory MultipleLaneChange::GetEgoTrajectory() const
{
    return ego_trajectory_;
}

void MultipleLaneChange::HandleFollowLaneState()
{
    const auto lane_change_trajectory{ComputeLaneChangeTrajectory(ego_state_, parameters_)};

    if (EgoCollidesWithObject(ego_state_, lane_change_trajectory, objects_) || EgoReachedTargetLane())
    {
        KeepFollowingLane();
    }
    else
    {
        StartLaneChange(lane_change_trajectory);
    }
}

bool MultipleLaneChange::EgoReachedTargetLane() const
{
    const auto target_lane_id{parameters_.number_of_lanes - 1};
    return (ego_state_.lane_id == target_lane_id);
}

void MultipleLaneChange::KeepFollowingLane()
{
    ego_trajectory_ = ComputeFollowLaneTrajectory(ego_state_, parameters_);
    UpdateEgoState(ego_trajectory_);
}

void MultipleLaneChange::StartLaneChange(const Trajectory& lane_change_trajectory)
{
    ego_trajectory_ = lane_change_trajectory;
    UpdateEgoState(ego_trajectory_);
    state_machine_.HandleEvent(MotionTransitions::kStartLaneChange);
}

void MultipleLaneChange::UpdateEgoState(const Trajectory& ego_trajectory)
{
    ego_state_.x_coordinate = ego_trajectory.at(0).x;
}

void MultipleLaneChange::HandleLaneChangeState()
{
    ConsumeLaneChangeTrajectory();

    if (LaneChangeTrajectoryFullyConsumed())
    {
        StartFollowingLane();
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

void MultipleLaneChange::StartFollowingLane()
{
    ++ego_state_.lane_id;
    state_machine_.HandleEvent(MotionTransitions::kLaneChangeCompleted);
}
