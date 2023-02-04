#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "turtlelib/diff_drive.hpp"
#include <string>

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry"), count_(0)
  {
    // Get odometry parameters
    auto body_id_desc = rcl_interfaces::msg::ParameterDescriptor();
    body_id_desc.description = "Name of the body frame of the robot";
    this->declare_parameter("body_id", body_id_desc);
    body_id = this->get_parameter("body_id").get_parameter_value().get<std::string>();
    if(!body_id){RCLCPP_ERROR_STREAM(this->get_logger(), "body_id not defined"); rclcpp::shutdown();}

    auto odom_id_desc = rcl_interfaces::msg::ParameterDescriptor();
    odom_id_desc.description = "Name of the odometetry frame of the robot";
    this->declare_parameter("odom_id", odom_id_desc);
    odom_id = this->get_parameter("odom_id").get_parameter_value().get<std::string>();

    auto wheel_left_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_left_desc.description = "Name of the left wheel joint of the robot";
    this->declare_parameter("wheel_left", wheel_left_desc);
    wheel_left = this->get_parameter("wheel_left").get_parameter_value().get<std::string>();

    auto wheel_right_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_right_desc.description = "Name of the right wheel joint of the robot";
    this->declare_parameter("wheel_right", wheel_right_desc);
    wheel_right = this->get_parameter("wheel_right").get_parameter_value().get<std::string>();
  }

private:
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
