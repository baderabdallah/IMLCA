#include <ros/ros.h>
#include <lane_msgs/Mlc.h>
#include "std_msgs/Float32MultiArray.h"

struct VehicleState
{
    int lane_id{0};
    double x_coordinate{0.0};
    double speed{0.0};  // km/h
};

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

class StateConverter
{
  private:
    ros::Subscriber mlc_subscriber_;
    ros::Publisher transformed_environment_publisher_;

  public:
    StateConverter(ros::NodeHandle *nh)
    {
        if (!init_) {
          initialize();
          init_ = true;
        }
        mlc_subscriber_ = nh->subscribe("/mlc/mlc_data", 10, &StateConverter::callbackMlcTopic, this);
        transformed_environment_publisher_ = nh->advertise<std_msgs::Float32MultiArray>("/mlc/transformed_state", 1);
    }

    void initialize() {
      transformed_environment_.layout.dim.push_back(std_msgs::MultiArrayDimension());
      transformed_environment_.layout.dim.push_back(std_msgs::MultiArrayDimension());
      transformed_environment_.layout.dim[0].label = "height";
      transformed_environment_.layout.dim[1].label = "width";
      transformed_environment_.layout.dim[0].size = 5;
      transformed_environment_.layout.dim[1].size = 5;
      transformed_environment_.layout.dim[0].stride = 5 * 5;
      transformed_environment_.layout.dim[1].stride = 5;
      transformed_environment_.layout.data_offset = 0;
      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
          transformed_environment_.data.push_back(static_cast<float>(0.0));
        }
      }
    }

    void publish_transformed_environment_() {
      transformed_environment_publisher_.publish(transformed_environment_);
    }

    void callbackMlcTopic(const lane_msgs::Mlc & msg)
    {
      const auto& scenario_data = msg.scenario_data;
      const auto& ego_info = msg.ego_info;
      const auto& ego_trajectory = msg.ego_vehicle_trajectory.trajectory;

      const auto objects{ConvertToInternalType(scenario_data)};

      double ego_lateral_speed = 0.0;
      double y_new = 14.0;
      if (ego_trajectory.size() > 0) {
        y_new = ego_trajectory[0].y;
      }
      y_new = 0.75 - (y_new-2)/(14-2) * 0.75;
      ego_lateral_speed = (y_new - transformed_environment_.data[2]) * 0.3;

      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
          transformed_environment_.data[i * 5 + j] = 0;
        }
      }

      transformed_environment_.data[0] = 1;
      transformed_environment_.data[1] = 1;
      transformed_environment_.data[2] = y_new;
      transformed_environment_.data[3] = ego_info.velocity_x / 90.0 * 0.375;
      transformed_environment_.data[4] = ego_lateral_speed;

      std::vector<VehicleState> filtered_vehicles{};
      std::copy_if(objects.begin(), objects.end(),
              std::back_inserter(filtered_vehicles),
              [ego_info = ego_info](const VehicleState& state) {
                return (state.x_coordinate - ego_info.pos_x + 10.0) > 0.0; });

      std::sort(filtered_vehicles.begin(), filtered_vehicles.end(), [](const VehicleState& state1, const VehicleState& state2){
        return state1.x_coordinate < state2.x_coordinate;
      });


      size_t counter = 1;
      for (const auto &obj : filtered_vehicles) {
        transformed_environment_.data[counter * 5 + 0] = 1;
        transformed_environment_.data[counter * 5 + 1] = (obj.x_coordinate - ego_info.pos_x - 6.0)/280;
        transformed_environment_.data[counter * 5 + 2] = (float)1.0/4 * obj.lane_id - transformed_environment_.data[2];
        transformed_environment_.data[counter * 5 + 3] = (obj.speed)/90.0 * 0.375 - transformed_environment_.data[3];
        transformed_environment_.data[counter * 5 + 4] = -transformed_environment_.data[4];
        counter ++;
        if (counter == 5) break;
      }
      transformed_environment_publisher_.publish(transformed_environment_);
    }


  private:
    bool init_ = false;
    std_msgs::Float32MultiArray transformed_environment_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_converter");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  StateConverter state_converter(&nh);
  ROS_INFO("state_converter is now started");
  ros::waitForShutdown();
}
