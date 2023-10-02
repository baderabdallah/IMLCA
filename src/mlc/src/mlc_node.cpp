#include <ros/ros.h>
#include <lane_msgs/Mlc.h>
#include <lane_msgs/ScenarioData.h>
#include "core/multiple_lane_change.h"
#include "core/parameters.h"


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

lane_msgs::Mlc ConvertToRosType(const lane_msgs::ScenarioData & msg, const Trajectory ego_trajectory)
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

   return mlc;
}

class Mlc
{
  private:
    ros::Subscriber scenario_subscriber_;
    ros::Publisher trajectory_publisher_;
    ros::Timer current_timer_;

  public:
    Mlc(ros::NodeHandle *nh)
    {
        float publish_frequency_ = 10.0;

        scenario_subscriber_ = nh->subscribe("/mlc/scenario_information", 10, &Mlc::callbackSubscriber, this);
        trajectory_publisher_ = nh->advertise<lane_msgs::Mlc>("/mlc/mlc_data", 10);
        current_timer_ = nh->createTimer(ros::Duration(1.0 / publish_frequency_), &Mlc::publish, this);
    }

    void publish(const ros::TimerEvent &event)
    {
        const auto ego_trajectory{multiple_lane_change_.GetEgoTrajectory()};
        const auto msg{ConvertToRosType(input_msg_, ego_trajectory)};

        ROS_INFO("Trajectory Generated");
        trajectory_publisher_.publish(msg);
    }

    void callbackSubscriber(const lane_msgs::ScenarioData & msg)
    {
        input_msg_ = msg;

        const auto objects{ConvertToInternalType(msg)};
        multiple_lane_change_.SetObjects(objects);
        multiple_lane_change_.Step();
    }


  private:
    Parameters parameters_{};
    MultipleLaneChange multiple_lane_change_{parameters_};
    lane_msgs::ScenarioData input_msg_{};
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
