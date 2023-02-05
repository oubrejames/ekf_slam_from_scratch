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
    this->declare_parameter("Hz", 200.0, hz_desc);
    int hz = this->get_parameter("Hz").get_parameter_value().get<double>();
    auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (hz)));
    timer_ = this->create_wall_timer(
      hz_in_ms, std::bind(&Circle::timer_callback, this));

    circle_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Define control service
    control_server_ = this->create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_server_cb, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    circle_pub_->publish(robot_twist);
  }

    void control_server_cb(
      const std::shared_ptr<nuturtle_control::srv::Control::Request> req,
      std::shared_ptr<nuturtle_control::srv::Control::Response>)
    {
      robot_twist.angular.z = req->velocity;
      robot_twist.linear.x = req->velocity*req->radius;
    }

    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr circle_pub_;
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_server_;
    double robot_vel, arc_radius;
    geometry_msgs::msg::Twist robot_twist;
};
//cmd vel publisher
//control service
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
