#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "turtlelib/diff_drive.hpp"
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <turtlesim/srv/spawn.hpp>
#include "nuturtle_control/srv/control.hpp"
#include <std_srvs/srv/empty.hpp>

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle"), count_(0)
  {
    // Create frequency parameter, convert it to chrono ms for timer, and create timer
    auto hz_desc = rcl_interfaces::msg::ParameterDescriptor{};
    hz_desc.description = "Frequency of the  timer in Hz";
    this->declare_parameter("frequency", 100.0, hz_desc);
    int hz = this->get_parameter("frequency").get_parameter_value().get<double>();
    auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (hz)));
    timer_ = this->create_wall_timer(
      hz_in_ms, std::bind(&Circle::timer_callback, this));

    circle_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Define control service
    control_server_ = this->create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_server_cb, this, std::placeholders::_1, std::placeholders::_2));

    // Define reverse server
    reverse_server_ = this->create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse, this, std::placeholders::_1, std::placeholders::_2));

    // Define stop server
    stop_server_ = this->create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    if (!stop_flag) {
      circle_pub_->publish(robot_twist);
    }
  }

  void control_server_cb(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> req,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    robot_twist.angular.z = req->velocity;
    robot_twist.linear.x = req->velocity * req->radius;
  }

  void reverse(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    robot_twist.angular.z *= -1.0;
    robot_twist.linear.x *= -1.0;
  }

  void stop(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    robot_twist.angular.z *= 0.0;
    robot_twist.linear.x *= 0.0;
    stop_flag = true;
  }

  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr circle_pub_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  double arc_radius = 0.0;
  geometry_msgs::msg::Twist robot_twist;
  bool stop_flag = false;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
