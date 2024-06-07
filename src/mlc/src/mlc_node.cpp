#include <ros/ros.h>
#include <lane_msgs/Mlc.h>
#include <lane_msgs/ScenarioData.h>
#include <lane_msgs/EgoInfo.h>
#include <std_msgs/String.h>
#include "core/multiple_lane_change.h"
#include "core/parameters.h"
#include <iostream>


std::vector<VehicleState> ConvertToInternalType(const lane_msgs::ScenarioData & msg)
{
    std::vector<VehicleState> objects{};
    const auto& ros_objects{msg.vehicles_information};

    for (const auto& ros_object : ros_objects)
    {
        VehicleState object{};
        object.lane_id = ros_object.lane_number;
        object.x_coordinate = ros_object.pos_x;
        object.speed = ros_object.velocity_x * 3.6; //kph
        objects.push_back(object);
    }

    return objects;
}

lane_msgs::Mlc ConvertToRosType(const lane_msgs::ScenarioData & msg, const Trajectory ego_trajectory, const EgoState ego_state)
{
   lane_msgs::Mlc mlc{};

    mlc.scenario_data = msg;

    for (const auto& trajectory_point : ego_trajectory)
    {
        lane_msgs::Pose pose{};
        pose.x = trajectory_point.x;
        pose.y = trajectory_point.y;

        mlc.ego_vehicle_trajectory.trajectory.push_back(pose);
    }

    lane_msgs::EgoInfo ego_info{};
    ego_info.lane_number = ego_state.lane_id;
    ego_info.pos_x = ego_state.x_coordinate;
    ego_info.velocity_x = ego_state.speed;

    mlc.ego_info = ego_info;

   return mlc;
}

class Mlc
{
  private:
    ros::Subscriber scenario_subscriber_;
    ros::Subscriber keyboard_subscriber_;
    ros::Publisher trajectory_publisher_;
    ros::Timer current_timer_;

  public:
    Mlc(ros::NodeHandle *nh)
    {
        float publish_frequency_ = 10.0;

        keyboard_subscriber_ = nh->subscribe("/keyboard_input", 10, &Mlc::callbackSubscriberKeyboard, this);
        scenario_subscriber_ = nh->subscribe("/mlc/scenario_information", 10, &Mlc::callbackSubscriberScenario, this);
        
        trajectory_publisher_ = nh->advertise<lane_msgs::Mlc>("/mlc/mlc_data", 10);
        current_timer_ = nh->createTimer(ros::Duration(1.0 / publish_frequency_), &Mlc::publish, this);

        if (!init_) {
          initialize();
          init_ = true;
        }

        multiple_lane_change_ = new MultipleLaneChange(parameters_, ego_state_);
    }

    void initialize() {
      ego_state_.speed = parameters_.ego_speed;
    }

    void publish(const ros::TimerEvent &event)
    {
        const auto ego_trajectory{multiple_lane_change_->GetEgoTrajectory()};
        const auto ego_state{multiple_lane_change_->GetEgoState()};
        const auto msg{ConvertToRosType(scenario_data_, ego_trajectory, ego_state)};

        ROS_INFO("Trajectory Generated");
        trajectory_publisher_.publish(msg);
    }

    void callbackSubscriberScenario(const lane_msgs::ScenarioData & msg)
    {
        scenario_data_ = msg;

        const auto objects{ConvertToInternalType(msg)};
        multiple_lane_change_->SetObjects(objects);
        multiple_lane_change_->Step();
    }
    void callbackSubscriberKeyboard(const std_msgs::String::ConstPtr& msg)
    {
      ROS_INFO("Received key: %s", msg->data.c_str());

      multiple_lane_change_->SetKeyboardInput(msg->data);
      
      return;
    }


  private:
    bool init_ = false;
    lane_msgs::ScenarioData scenario_data_{};
    const Parameters parameters_{};
    EgoState ego_state_{};
    MultipleLaneChange* multiple_lane_change_= nullptr;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mlc");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  Mlc Mlc(&nh);
  ROS_INFO("mlc is now started");
  ros::waitForShutdown();
}
