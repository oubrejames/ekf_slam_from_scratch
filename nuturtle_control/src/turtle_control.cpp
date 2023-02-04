#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control"), count_(0)
  {
    // Get turtle description parameters and throw errors if they are not provided
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

    // Create DiffDrive object 
    turtlebot = {track_width, wheel_radius};

    // Create cmd_vel subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::velocity_callback, this, std::placeholders::_1));

    // Create publisher to publish wheel commands
    wheel_cmd_pub_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_commands", 10);
  }

private:
    /// @brief subscribe to the cmd_vel topic to get robot velocity commands (rads/s)
    /// @param msg the Twist from the cmd_vel topic
    void velocity_callback(const geometry_msgs::msg::Twist & msg)
    { 
        velocity_command = msg;
        double w = msg.angular.z;
        double x = msg.angular.x;
        double y = msg.angular.y;
        turtlelib::WheelPos wheel_command = turtlebot.inverse_kinematics({w, x, y});
        nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
        wheel_cmd_msg.left_velocity = wheel_command.l;
        wheel_cmd_msg.right_velocity = wheel_command.r;
        wheel_cmd_pub_->publish(wheel_cmd_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec;
    double encoder_ticks_per_rad, collision_radius;
    size_t count_;
    geometry_msgs::msg::Twist velocity_command;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
    turtlelib::DiffDrive turtlebot = {0.0, 0.0};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
