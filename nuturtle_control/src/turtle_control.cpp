#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <std_srvs/srv/empty.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
// #include "nusim/srv/teleport.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control"), count_(0)
  {
    // Get turtle description parameters
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_radius_desc.description = "Radius of turtlebot wheel";
    this->declare_parameter("wheel_radius", -1.0, wheel_radius_desc);
    wheel_radius = this->get_parameter("wheel_radius").get_parameter_value().get<double>();
    if(wheel_radius == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "Wheel radius not defined"); rclcpp::shutdown();}

    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor();
    track_width_desc.description = "Trackwidth between turtlebot wheels";
    this->declare_parameter("track_width", -1.0, track_width_desc);
    track_width = this->get_parameter("track_width").get_parameter_value().get<double>();
    if(track_width == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "Track width not defined"); rclcpp::shutdown();}

    auto motor_cmd_max_desc = rcl_interfaces::msg::ParameterDescriptor();
    motor_cmd_max_desc.description = "Maximium wheel velocity command";
    this->declare_parameter("motor_cmd_max", -1.0, motor_cmd_max_desc);
    motor_cmd_max = this->get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    if(motor_cmd_max == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_max not defined"); rclcpp::shutdown();}

    auto motor_cmd_per_rad_sec_desc = rcl_interfaces::msg::ParameterDescriptor();
    motor_cmd_per_rad_sec_desc.description = "Maximium wheel velocity command in ticks per rad/sec";
    this->declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_desc);
    motor_cmd_per_rad_sec = this->get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    if(motor_cmd_per_rad_sec == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_per_rad_sec not defined"); rclcpp::shutdown();}

    auto encoder_ticks_per_rad_desc = rcl_interfaces::msg::ParameterDescriptor();
    encoder_ticks_per_rad_desc.description = "Ticks per second of the encoder";
    this->declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_desc);
    encoder_ticks_per_rad = this->get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    if(encoder_ticks_per_rad == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "encoder_ticks_per_rad not defined"); rclcpp::shutdown();}

    auto collision_radius_desc = rcl_interfaces::msg::ParameterDescriptor();
    collision_radius_desc.description = "Radius of turtlebot wheel";
    this->declare_parameter("collision_radius", -1.0, collision_radius_desc);
    collision_radius = this->get_parameter("collision_radius").get_parameter_value().get<double>();
    if(collision_radius == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "collision_radius not defined"); rclcpp::shutdown();}

  }

private:
    double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec;
    double encoder_ticks_per_rad, collision_radius;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
