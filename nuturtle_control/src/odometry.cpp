/// \file
/// \brief This file contains the odometry node which calculates odometry and publsihes odometry information
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
///     \param hz_in_ms (double): Frequency of the timer in Hz
///     \param body_id (string): Name of the body frame of the robot
///     \param odom_id (string): Name of the odometetry frame of the robot
///     \param wheel_left (string): Name of the left wheel joint of the robot
///     \param wheel_right (string): Name of the right wheel joint of the robot
///     \param wheel_radius (double): Radius of turtlebot wheel
///     \param track_width (double): Trackwidth between turtlebot wheels
///
/// PUBLISHES:
///     topic_name (topic_type): description of topic
///     /odom (nav_msgs::msg::Odometry): Publishes the odometry information of the robot
///     ~/path (nav_msgs::msg::Path): Publishes the path of where the robot has driven
///
/// SUBSCRIBES:
///     topic_name (topic_type): description of topic
///     /joint_states (sensor_msgs::msg::JointState): Subcribes to the joint states of the robot
///
/// SERVERS:
///     initial_pose (nuturtle_control::srv::Spawn): spawns the Turtlebot at a specified position

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
#include "nuturtle_control/srv/spawn.hpp"
#include <nav_msgs/msg/path.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry"), count_(0)
  {
    // Create frequency parameter, convert it to chrono ms for timer, and create timer
    auto hz_desc = rcl_interfaces::msg::ParameterDescriptor{};
    hz_desc.description = "Frequency of the  timer in Hz";
    declare_parameter("frequency", 100.0, hz_desc);
    int hz = get_parameter("frequency").get_parameter_value().get<double>();
    auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (hz)));
    timer_ = create_wall_timer(
      hz_in_ms, std::bind(&Odometry::timer_callback, this));

    // Get odometry parameters
    auto body_id_desc = rcl_interfaces::msg::ParameterDescriptor();
    body_id_desc.description = "Name of the body frame of the robot";
    declare_parameter("body_id", "", body_id_desc);
    body_id = get_parameter("body_id").get_parameter_value().get<std::string>();
    if (body_id == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "body_id not defined"); rclcpp::shutdown();
    }

    auto odom_id_desc = rcl_interfaces::msg::ParameterDescriptor();
    odom_id_desc.description = "Name of the odometetry frame of the robot";
    declare_parameter("odom_id", "odom", odom_id_desc);
    odom_id = get_parameter("odom_id").get_parameter_value().get<std::string>();

    auto wheel_left_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_left_desc.description = "Name of the left wheel joint of the robot";
    declare_parameter("wheel_left", "", wheel_left_desc);
    wheel_left = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    if (wheel_left == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_left not defined"); rclcpp::shutdown();
    }

    auto wheel_right_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_right_desc.description = "Name of the right wheel joint of the robot";
    declare_parameter("wheel_right", "", wheel_right_desc);
    wheel_right = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    if (wheel_right == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_right not defined"); rclcpp::shutdown();
    }

    // Get turtle description parameters
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_radius_desc.description = "Radius of turtlebot wheel";
    declare_parameter("wheel_radius", -1.0, wheel_radius_desc);
    wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor();
    track_width_desc.description = "Trackwidth between turtlebot wheels";
    declare_parameter("track_width", -1.0, track_width_desc);
    track_width = get_parameter("track_width").get_parameter_value().get<double>();

    // Create publisher to publish odometry
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;

    // Create initial odom msg to broadcast
    // Initialize internal odom
    internal_odom = turtlelib::DiffDrive{track_width, wheel_radius};
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_pos.theta);

    // Populate odom position with current position
    odom_msg.pose.pose.position.x = current_pos.x;
    odom_msg.pose.pose.position.y = current_pos.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Populate odom twist with body twist
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create joint_states subscriber
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_states_cb, this, std::placeholders::_1));

    // Define initial_pose service
    initial_pose_server_ = create_service<nuturtle_control::srv::Spawn>(
      "initial_pose",
      std::bind(&Odometry::initial_pose_cb, this, std::placeholders::_1, std::placeholders::_2));

    // Define path header
    visited_path.header.frame_id = "nusim/world";
    visited_path.header.stamp = get_clock()->now();

    // Define path publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/path",
      10);
  }

private:
  void timer_callback()
  {

    // Add point to path and publish
    if (time_count % 100 == 0) {
      visited_path.header.stamp = get_clock()->now();
      visited_path.poses.push_back(
        create_pose_stamped(
          current_pos.x, current_pos.y,
          current_pos.theta));
      path_pub_->publish(visited_path);
    }
    time_count++;
  }

  geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double theta)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "red/base_footprint";
    pose_stamped.header.stamp = get_clock()->now();
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;

    q.setRPY(0.0, 0.0, theta);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    return pose_stamped;
  }

  /// @brief
  /// @param msg
  void joint_states_cb(const sensor_msgs::msg::JointState & msg)
  {
    // Update interal odom
    turtlelib::WheelPos new_wp =
    {msg.position.at(0) - prev_wheel_pos.r, msg.position.at(1) - prev_wheel_pos.l};
    prev_wheel_pos.r = msg.position.at(0);
    prev_wheel_pos.l = msg.position.at(1);

    // Get body twist from current wheel positions and update current position of robot
    turtlelib::Twist2D Vb = internal_odom.forward_kinematics(new_wp);

    // Update odom message
    odom_msg.header.stamp = this->get_clock()->now();

    // Convert current orientation to quaternian
    current_pos = internal_odom.get_current_pos();
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pos.theta);

    // Populate odom position with current position
    odom_msg.pose.pose.position.x = current_pos.x;
    odom_msg.pose.pose.position.y = current_pos.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Populate odom twist with body twist
    odom_msg.twist.twist.linear.x = Vb.x;
    odom_msg.twist.twist.linear.y = Vb.y;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = Vb.w;

    // Publish updated odometry
    odom_pub_->publish(odom_msg);
    broadcast_tf();

  }

  void broadcast_tf()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;

    t.transform.translation.x = current_pos.x;
    t.transform.translation.y = current_pos.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pos.theta);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
  }

  void initial_pose_cb(
    const std::shared_ptr<nuturtle_control::srv::Spawn::Request> req,
    std::shared_ptr<nuturtle_control::srv::Spawn::Response>)
  {
    internal_odom =
    {track_width, wheel_radius, {req->x, req->y, req->theta},
      {internal_odom.get_current_wheel_pos()}};
  }

  std::string body_id, odom_id, wheel_left, wheel_right;
  double wheel_radius, track_width;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_msg;
  turtlelib::DiffDrive internal_odom = {1.0, 1.0};
  size_t count_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::Spawn>::SharedPtr initial_pose_server_;
  turtlelib::RobotConfig current_pos = {0.0, 0.0, 0.0};
  turtlelib::WheelPos prev_wheel_pos = {0.0, 0.0};
  double dt_time = 0.005;
  int time_count = 0;
  nav_msgs::msg::Path visited_path;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  tf2::Quaternion q;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
