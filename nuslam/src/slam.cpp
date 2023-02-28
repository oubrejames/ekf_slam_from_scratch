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
#include "turtlelib/slam.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "turtlelib/rigid2d.hpp"

using namespace std::chrono_literals;

class SlamNode : public rclcpp::Node
{
public:
  SlamNode()
  : Node("slam_node"), count_(0)
  {
    // Create frequency parameter, convert it to chrono ms for timer, and create timer
    auto hz_desc = rcl_interfaces::msg::ParameterDescriptor{};
    hz_desc.description = "Frequency of the  timer in Hz";
    this->declare_parameter("frequency", 5.0, hz_desc);
    int hz = this->get_parameter("frequency").get_parameter_value().get<double>();
    auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (hz)));
    timer_ = this->create_wall_timer(
      hz_in_ms, std::bind(&SlamNode::timer_callback, this));

    // Get odometry parameters
    auto body_id_desc = rcl_interfaces::msg::ParameterDescriptor();
    body_id_desc.description = "Name of the body frame of the robot";
    this->declare_parameter("body_id", "", body_id_desc);
    body_id = this->get_parameter("body_id").get_parameter_value().get<std::string>();
    if (body_id == "") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "body_id not defined"); rclcpp::shutdown();
    }

    auto odom_id_desc = rcl_interfaces::msg::ParameterDescriptor();
    odom_id_desc.description = "Name of the odometetry frame of the robot";
    this->declare_parameter("odom_id", "odom", odom_id_desc);
    odom_id = this->get_parameter("odom_id").get_parameter_value().get<std::string>();

    auto wheel_left_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_left_desc.description = "Name of the left wheel joint of the robot";
    this->declare_parameter("wheel_left", "", wheel_left_desc);
    wheel_left = this->get_parameter("wheel_left").get_parameter_value().get<std::string>();
    if (wheel_left == "") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_left not defined"); rclcpp::shutdown();
    }

    auto wheel_right_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_right_desc.description = "Name of the right wheel joint of the robot";
    this->declare_parameter("wheel_right", "", wheel_right_desc);
    wheel_right = this->get_parameter("wheel_right").get_parameter_value().get<std::string>();
    if (wheel_right == "") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_right not defined"); rclcpp::shutdown();
    }

    // Get turtle description parameters
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_radius_desc.description = "Radius of turtlebot wheel";
    this->declare_parameter("wheel_radius", -1.0, wheel_radius_desc);
    wheel_radius = this->get_parameter("wheel_radius").get_parameter_value().get<double>();

    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor();
    track_width_desc.description = "Trackwidth between turtlebot wheels";
    this->declare_parameter("track_width", -1.0, track_width_desc);
    track_width = this->get_parameter("track_width").get_parameter_value().get<double>();

      // Create publisher to publish odometry
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      odom_msg.header.stamp = this->get_clock()->now();
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
    tf_broadcaster2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create joint_states subscriber
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "blue/joint_states", 10, std::bind(&SlamNode::joint_states_cb, this, std::placeholders::_1));

    // Define initial_pose service
    initial_pose_server_ = this->create_service<nuturtle_control::srv::Spawn>(
      "initial_pose",
      std::bind(&SlamNode::initial_pose_cb, this, std::placeholders::_1, std::placeholders::_2));

    // Define path header
    visited_path.header.frame_id = "green/odom";
    visited_path.header.stamp = get_clock()->now();

    // Define path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "~/path",
      10);

    // Create publisher to publish joint states
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);

    // Define parameter to change basic sensor noise
    auto basic_sensor_variance_desc = rcl_interfaces::msg::ParameterDescriptor();
    basic_sensor_variance_desc.description = "Variance to change sensor noise for basic sensor";
    declare_parameter("basic_sensor_variance", 0.01, basic_sensor_variance_desc);
    basic_sensor_variance = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();

    ekf_slam = {0.001, basic_sensor_variance};

    // Create subcriber to get actual obstacle positions
    sensed_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "basic_sensor/marker_array", 10, std::bind(&SlamNode::sensed_obstacles_cb, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
  

      visited_path.header.stamp = get_clock()->now();
      visited_path.poses.push_back(create_pose_stamped(current_pos.x, current_pos.y, current_pos.theta));
      path_pub_->publish(visited_path);

  }

  geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double theta){
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "green/base_footprint";
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
    // Publish joint states
    joint_state_pub_->publish(msg);

    // Update interal odom
    turtlelib::WheelPos new_wp =
    {msg.position.at(0) - prev_wheel_pos.r, msg.position.at(1) - prev_wheel_pos.l};
    prev_wheel_pos.r = msg.position.at(0);
    prev_wheel_pos.l = msg.position.at(1);

    // Get body twist from current wheel positions and update current position of robot
    Vb = internal_odom.forward_kinematics(new_wp);

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
    // odom_pub_->publish(odom_msg);
    broadcast_tf();

  }

  void broadcast_tf()
  {
    geometry_msgs::msg::TransformStamped Tor;

    Tor.header.stamp = this->get_clock()->now();
    Tor.header.frame_id = odom_id;
    Tor.child_frame_id = body_id;

    Tor.transform.translation.x =  current_pos.x;
    Tor.transform.translation.y = current_pos.y;
    Tor.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pos.theta);
    Tor.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(Tor);

    turtlelib::Transform2D Tor_tmp{{current_pos.x, current_pos.y}, current_pos.theta};
    turtlelib::Transform2D Tmr{{slam_pos.x, slam_pos.y}, slam_pos.theta};
    turtlelib::Transform2D Tmo_calc = Tmr*Tor_tmp.inv();
    ////////////////
    geometry_msgs::msg::TransformStamped Tmo;
    Tmo.header.stamp = this->get_clock()->now();
    Tmo.header.frame_id = "map";
    Tmo.child_frame_id = odom_id;

    Tmo.transform.translation.x =  Tmo_calc.translation().x;
    Tmo.transform.translation.y = Tmo_calc.translation().y;
    Tmo.transform.translation.z = 0.0;

    tf2::Quaternion q2;
    q2.setRPY(0, 0, Tmo_calc.rotation());
    Tmo.transform.rotation = tf2::toMsg(q2);

    tf_broadcaster2_->sendTransform(Tmo);
  }

  void initial_pose_cb(
    const std::shared_ptr<nuturtle_control::srv::Spawn::Request> req,
    std::shared_ptr<nuturtle_control::srv::Spawn::Response>)
  {
    internal_odom =
    {track_width, wheel_radius, {req->x, req->y, req->theta},
      {internal_odom.get_current_wheel_pos()}};
  }

    // Subscribe to get the sensed obstacle positions
    void sensed_obstacles_cb(const visualization_msgs::msg::MarkerArray & msg){
        sensed_obstacles = msg;
        current_pos2 = internal_odom.get_current_pos();

        ekf_slam.predict({current_pos2.theta, current_pos2.x, current_pos2.y});
        // RCLCPP_ERROR_STREAM(this->get_logger(), "PREDICT thing " << ekf_slam.get_belief_predict());

        for(size_t j =0; j < sensed_obstacles.markers.size(); j++){
          if(sensed_obstacles.markers.at(j).action < 2){
            RCLCPP_ERROR_STREAM(this->get_logger(), "j " << j << " x " << sensed_obstacles.markers.at(j).pose.position.x << " x " << sensed_obstacles.markers.at(j).pose.position.y);

            ekf_slam.update({sensed_obstacles.markers.at(j).pose.position.x, sensed_obstacles.markers.at(j).pose.position.y, static_cast<int>(j)});
            // RCLCPP_ERROR_STREAM(this->get_logger(), "m " << ekf_slam.M_output);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "rj " << ekf_slam.rj_out);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "phi_j " << ekf_slam.phi_j_out);

            arma::colvec tmp_pose = ekf_slam.get_belief();
            slam_pos.x = tmp_pose(1);
            slam_pos.y = tmp_pose(2);
            slam_pos.theta = tmp_pose(0);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "A " << std::endl << ekf_slam.A_out << std::endl);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "sigma_t_pred_get " << std::endl << ekf_slam.sigma_t_pred_get << std::endl);

            RCLCPP_ERROR_STREAM(this->get_logger(), "cov_out " << std::endl << ekf_slam.cov_out << std::endl);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "A " << std::endl << ekf_slam.A_out << std::endl);

            // RCLCPP_ERROR_STREAM(this->get_logger(), "K " << std::endl << ekf_slam.K_out << std::endl);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "whole thing " << ekf_slam.get_belief());
            // RCLCPP_ERROR_STREAM(this->get_logger(), "zout " << ekf_slam.z_out << std::endl);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "zes " << ekf_slam.z_es << std::endl);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "zdiff " << ekf_slam.z_diff << std::endl);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "phi_j_es " << ekf_slam.phi_j_out << std::endl);

            // RCLCPP_ERROR_STREAM(this->get_logger(), "obstacle_tracking " << ekf_slam.obstacle_tracking);

        }}

        // RCLCPP_ERROR_STREAM(this->get_logger(), "slam_pos.x " << slam_pos.x);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "slam_pos.y " << slam_pos.y);

        RCLCPP_ERROR_STREAM(this->get_logger(), "whole thing " << ekf_slam.get_belief());
        

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
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster2_;

  rclcpp::Service<nuturtle_control::srv::Spawn>::SharedPtr initial_pose_server_;
  turtlelib::RobotConfig current_pos = {0.0, 0.0, 0.0};
    turtlelib::RobotConfig current_pos2 = {0.0, 0.0, 0.0};
  turtlelib::RobotConfig slam_pos = {0.0, 0.0, 0.0};

  turtlelib::WheelPos prev_wheel_pos = {0.0, 0.0};
  double dt_time = 0.005;
  int time_count = 0;
  nav_msgs::msg::Path visited_path;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  tf2::Quaternion q;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  turtlelib::EKFSlam ekf_slam;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensed_obstacles_sub_;
  double basic_sensor_variance = 0.0;
  visualization_msgs::msg::MarkerArray sensed_obstacles;
  turtlelib::Twist2D Vb = {0.0, 0.0, 0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNode>());
  rclcpp::shutdown();
  return 0;
}
