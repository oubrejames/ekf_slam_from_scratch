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
#include "nusim/srv/teleport.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <random>
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;

class NusimNode : public rclcpp::Node
{
public:
  NusimNode()
  : Node("nusim"), count_(0)
  {
    // Get turtle description parameters and throw errors if they are not provided
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor();
    wheel_radius_desc.description = "Radius of turtlebot wheel";
    declare_parameter("wheel_radius", -1.0, wheel_radius_desc);
    wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    if (wheel_radius == -1.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Wheel radius not defined"); rclcpp::shutdown();
    }

    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor();
    track_width_desc.description = "Trackwidth between turtlebot wheels";
    declare_parameter("track_width", -1.0, track_width_desc);
    track_width = get_parameter("track_width").get_parameter_value().get<double>();
    if (track_width == -1.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Track width not defined"); rclcpp::shutdown();
    }

    auto motor_cmd_max_desc = rcl_interfaces::msg::ParameterDescriptor();
    motor_cmd_max_desc.description = "Maximium wheel velocity command in ticks/sec";
    declare_parameter("motor_cmd_max", -1, motor_cmd_max_desc);
    motor_cmd_max = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    if (motor_cmd_max == -1.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_max not defined"); rclcpp::shutdown();
    }

    auto motor_cmd_per_rad_sec_desc = rcl_interfaces::msg::ParameterDescriptor();
    motor_cmd_per_rad_sec_desc.description = "Maximium wheel velocity command in ticks per rad/sec";
    declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_desc);
    motor_cmd_per_rad_sec =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    if (motor_cmd_per_rad_sec == -1.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_per_rad_sec not defined"); rclcpp::shutdown();
    }

    auto encoder_ticks_per_rad_desc = rcl_interfaces::msg::ParameterDescriptor();
    encoder_ticks_per_rad_desc.description = "Ticks per second of the encoder";
    declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_desc);
    encoder_ticks_per_rad =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    if (encoder_ticks_per_rad == -1.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "encoder_ticks_per_rad not defined"); rclcpp::shutdown();
    }

    auto collision_radius_desc = rcl_interfaces::msg::ParameterDescriptor();
    collision_radius_desc.description = "Radius of turtlebot wheel";
    declare_parameter("collision_radius", -1.0, collision_radius_desc);
    collision_radius = get_parameter("collision_radius").get_parameter_value().get<double>();
    if (collision_radius == -1.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "collision_radius not defined"); rclcpp::shutdown();
    }


    // Define arena wall parameters
    auto arena_x_len_desc = rcl_interfaces::msg::ParameterDescriptor();
    arena_x_len_desc.description = "Length of arena in the x direction";
    declare_parameter("~x_length", 3.5, arena_x_len_desc);
    arena_x_len = get_parameter("~x_length").get_parameter_value().get<double>();

    auto arena_y_len_desc = rcl_interfaces::msg::ParameterDescriptor();
    arena_y_len_desc.description = "Length of arena in the y direction";
    declare_parameter("~y_length", 5.0, arena_y_len_desc);
    arena_y_len = get_parameter("~y_length").get_parameter_value().get<double>();


    // Define a DiffDrive object to represent the turtlebot
    turtlebot = turtlelib::DiffDrive{track_width, wheel_radius};


    // Publisher to publish current timestep
    time_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Create frequency parameter, convert it to chrono ms for timer, and create timer
    auto hz_desc = rcl_interfaces::msg::ParameterDescriptor{};
    hz_desc.description = "Frequency of the  timer in Hz";
    declare_parameter("Hz", 200.0, hz_desc);
    int hz = get_parameter("Hz").get_parameter_value().get<double>();
    auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (hz)));
    timer_ = create_wall_timer(
      hz_in_ms, std::bind(&NusimNode::timer_callback, this));

    // Calculate dt for later calculations
    dt_time = 1.0 / (double)hz;


    // Define reset server
    reset_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&NusimNode::reset, this, std::placeholders::_1, std::placeholders::_2));


    // Define transform broadcaster to be used between nusim world and red robot
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // Define teleport service
    teleport_server_ = this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&NusimNode::teleport, this, std::placeholders::_1, std::placeholders::_2));


    // Create parameters for turtlebot initial position
    auto x0_desc = rcl_interfaces::msg::ParameterDescriptor();
    x0_desc.description = "Initial x position of turtlebot";
    declare_parameter("x0", 0.3, x0_desc);
    turtle_x0 = get_parameter("x0").get_parameter_value().get<double>();
    turtle_x = turtle_x0;

    auto y0_desc = rcl_interfaces::msg::ParameterDescriptor();
    y0_desc.description = "Initial y position of turtlebot";
    declare_parameter("y0", 0.0, y0_desc);
    turtle_y0 = get_parameter("y0").get_parameter_value().get<double>();
    turtle_y = turtle_y0;

    auto theta0_desc = rcl_interfaces::msg::ParameterDescriptor();
    theta0_desc.description = "Initial theta position of turtlebot";
    declare_parameter("theta0", 0.0, theta0_desc);
    turtle_theta0 = get_parameter("theta0").get_parameter_value().get<double>();
    turtle_theta = turtle_theta0;


    // Create parameters for obstacle position and size
    auto ob_x_desc = rcl_interfaces::msg::ParameterDescriptor();
    ob_x_desc.description = "List of x coordinates of obstacles";
    declare_parameter("obstacles/x", std::vector<double> {}, ob_x_desc);
    obstacles_x =
      get_parameter("obstacles/x").get_parameter_value().get<std::vector<double>>();

    auto ob_y_desc = rcl_interfaces::msg::ParameterDescriptor();
    ob_y_desc.description = "List of y coordinates of obstacles";
    declare_parameter("obstacles/y", std::vector<double> {}, ob_y_desc);
    obstacles_y =
      get_parameter("obstacles/y").get_parameter_value().get<std::vector<double>>();

    auto ob_r_desc = rcl_interfaces::msg::ParameterDescriptor();
    ob_r_desc.description = "Radius of cyindrical of obstacles";
    declare_parameter("obstacles/r", 0.05, ob_r_desc);
    obstacles_r = get_parameter("obstacles/r").get_parameter_value().get<double>();


    // Define a publisher to publish obstacle and wall markers
    obstacle_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      10);

    wall_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      10);


    // Only create obstacle marker array if there are actually obstacles defines
    if (!obstacles_x.empty()) {
      marker_array = make_obstacle_array( obstacles_x, obstacles_y);
    }

    // Make arena
    arena_marker_array = make_wall_array(
      std::vector<double> {arena_x_len / 2.0, 0.0,
        -arena_x_len / 2.0, 0.0},
      std::vector<double> {0.0, arena_y_len / 2, 0.0, -arena_y_len / 2});

    // Report error and kill node if obstacle x and y arrays are not same length
    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_ERROR(
        get_logger(),
        "obstacles_x and obstacles_y lists have different lengths, shutting down.");
      rclcpp::shutdown();
      return;
    }


    // Create wheel command subscriber
    wheel_commands_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&NusimNode::wheel_commands_cb, this, std::placeholders::_1));


    // Create publisher to publish sensor data
    sensor_data_pub_ = this->create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);

    // Define path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "~/path",
      10);

    // Define path header
    visited_path.header.frame_id = "nusim/world";
    visited_path.header.stamp = get_clock()->now();

    // Add noise parameter
    auto input_noise_desc = rcl_interfaces::msg::ParameterDescriptor();
    input_noise_desc.description = "Input noise to simulate sensor noise";
    this->declare_parameter("input_noise", 0.0, input_noise_desc);
    input_noise = this->get_parameter("input_noise").get_parameter_value().get<double>();

    // Add slip fraction
    auto slip_fraction_desc = rcl_interfaces::msg::ParameterDescriptor();
    slip_fraction_desc.description = "Fraction to simulate wheel slippage";
    this->declare_parameter("slip_fraction", 0.01, slip_fraction_desc);
    slip_fraction = this->get_parameter("slip_fraction").get_parameter_value().get<double>();

    // temp point pub to get heading
    heading_pub_ = this->create_publisher<geometry_msgs::msg::Point>("heading", 10);

    // Add parameter to make nusim only draw obstacles and walls
    auto draw_only_desc = rcl_interfaces::msg::ParameterDescriptor();
    draw_only_desc.description = "Radius of turtlebot wheel";
    declare_parameter("draw_only", false, draw_only_desc);
    draw_only = get_parameter("draw_only").get_parameter_value().get<bool>();
  }

private:
  uint64_t timestep = 0;
  geometry_msgs::msg::TransformStamped t;
  tf2::Quaternion q, q1;
  double turtle_x0, turtle_y0, turtle_theta0;
  double turtle_x, turtle_y, turtle_theta;
  double obstacles_r;
  std::vector<double> obstacles_x, obstacles_y;


  /// @brief create a marker array to publish obstacle cylinders
  /// @param x_coords x coordinates of the obstacles
  /// @param y_coords y coordinates of the obstacles
  /// @return maker area containing obstacle markers
  visualization_msgs::msg::MarkerArray make_obstacle_array(
    std::vector<double> x_coords,
    std::vector<double> y_coords)
  {
    // Initialize marker array
    visualization_msgs::msg::MarkerArray mkr_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
    for (size_t i = 0; i < (x_coords.size()); i++) {
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = obstacles_r * 2;
      marker.scale.y = obstacles_r * 2;
      marker.scale.z = 0.25;
      marker.pose.position.x = x_coords.at(i);
      marker.pose.position.y = y_coords.at(i);
      marker.pose.position.z = 0.25 / 2;

      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      mkr_array.markers.push_back(marker);
    }
    return mkr_array;
  }


  /// @brief create a marker array to publish arena walls
  /// @param x_coords x coordinates of the obstacles
  /// @param y_coords y coordinates of the obstacles
  /// @return maker area containing wall markers
  visualization_msgs::msg::MarkerArray make_wall_array(
    std::vector<double> x_coords,
    std::vector<double> y_coords)
  {
    double yaw = 0.0;
    // Initialize marker array
    visualization_msgs::msg::MarkerArray mkr_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();

    // Create X markers
    for (size_t i = 0; i < (x_coords.size()); i++) {
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      if(i % 2 == 0){
        // Create Y markers
        marker.scale.x = 0.2;
        marker.scale.y = arena_y_len;
        q.setRPY(0.0, 0.0, yaw);
        marker.scale.z = 0.25;
        marker.pose.position.x = x_coords.at(i);
        marker.pose.position.y = y_coords.at(i);
        marker.pose.position.z = 0.25 / 2;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
      }
      else {
        // Create X markers
        marker.scale.x = arena_x_len;
        marker.scale.y = 0.2;
        q.setRPY(0.0, 0.0, yaw);
        marker.scale.z = 0.25;
        marker.pose.position.x = x_coords.at(i);
        marker.pose.position.y = y_coords.at(i);
        marker.pose.position.z = 0.25 / 2;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
      }
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      mkr_array.markers.push_back(marker);
    }

    // Change marker angle by 90 degrees to make X and Y walls perpendicular
    yaw += turtlelib::PI / 2;

    return mkr_array;
  }

  /// @brief Teleport the turtlebot from one location to another
  /// @param req Custom srv containing a x,y,theta position
  /// @param - service response (empty)
  void teleport(
    const std::shared_ptr<nusim::srv::Teleport::Request> req,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    // Update robot's current position with service request input position
    turtle_x = req->x;
    turtle_y = req->y;
    turtle_theta = req->theta;
  }

  /// @brief reset timestep to 0 and return turtlebot to original location
  /// @param - service request (empty)
  /// @param - service response (empty)
  void reset(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep = 0;
    turtle_x = turtle_x0;
    turtle_y = turtle_y0;
    turtle_theta = turtle_theta0;
  }

  void timer_callback()
  {
    if(!draw_only){
      // Create message and publish timestep
      auto time_step_msg = std_msgs::msg::UInt64();
      time_step_msg.data = timestep++;
      time_publisher_->publish(time_step_msg);

      // Update TF time stamp and declare frames
      t.header.stamp = get_clock()->now();
      t.header.frame_id = "nusim/world";
      t.child_frame_id = "red/base_footprint";

      // Update world to robot transformation to turtlebot current location
      t.transform.translation.x = turtle_x;
      t.transform.translation.y = turtle_y;
      q1.setRPY(0.0, 0.0, turtle_theta);
      t.transform.rotation.x = q1.x();
      t.transform.rotation.y = q1.y();
      t.transform.rotation.z = q1.z();
      t.transform.rotation.w = q1.w();

      heading.z = turtle_theta;
      heading_pub_->publish(heading);

      // Send transformation
      tf_broadcaster_->sendTransform(t);
      obstacle_publisher->publish(marker_array);
      wall_publisher->publish(arena_marker_array);

      // Update current config of robot based on wheel commands
      double delta_wheel_pos_r = (dt_time * phi_r_rad_s);
      double delta_wheel_pos_l = (dt_time * phi_l_rad_s);

      // Apply wheel slippage if slip fraction specified
      if(slip_fraction > 0.0 && !turtlelib::almost_equal(delta_wheel_pos_l, 0.0) && !turtlelib::almost_equal(delta_wheel_pos_r, 0.0)){
        // Create a zero mean Gaussian distribution with a variance equal to the input nosie
        std::uniform_real_distribution<> uni_dist(-slip_fraction, slip_fraction);

        // Update the motor velocities to include the noise
        delta_wheel_pos_r += uni_dist(get_random());
        delta_wheel_pos_l += uni_dist(get_random());
      }

      // Save previous pose
      auto prev_pos = turtlebot.get_current_pos();;

      // Compute forward kinematics to update the robot's current position based off of wheel commands
      turtlebot.forward_kinematics({delta_wheel_pos_r, delta_wheel_pos_l});

      // Use getter to obtain robots current configuration
      current_pos = turtlebot.get_current_pos();
    
      // Check if you're current movement makes you collide with obstacle or wall
      if(!obstacle_collision(current_pos) && !wall_collision(current_pos)){
        // If you are not colliding with a wall or obstacle, update position as normal
        // Update turtlebot position for broadcasting its transformation
        turtle_x = current_pos.x;
        turtle_y = current_pos.y;
        turtle_theta = current_pos.theta;
      }
      else{
        // If you are colliding, do not update turtle bot TF and reset turtlebots location to prev
        turtlebot.set_current_pos(prev_pos);
      }

      // update sensor data
      // Update the encoder readings by incrementing new position plus the old position and converting 
      // it to encoder ticks
      // Update the previous wheel position
      sensor_readings.left_encoder = (delta_wheel_pos_l + prev_wheel_pos.l) * encoder_ticks_per_rad;
      sensor_readings.right_encoder = (delta_wheel_pos_r + prev_wheel_pos.r) * encoder_ticks_per_rad;
      prev_wheel_pos.l += delta_wheel_pos_l;
      prev_wheel_pos.r += delta_wheel_pos_r;


      // publish sensor data
      sensor_readings.stamp = get_clock()->now();
      sensor_data_pub_->publish(sensor_readings);

      // Add point to path and publish
      if (timestep % 100 == 0){
        visited_path.header.stamp = get_clock()->now();
        visited_path.poses.push_back(create_pose_stamped(turtle_x, turtle_y, turtle_theta));
        path_pub_->publish(visited_path);
      }
    }
    else{
      obstacle_publisher->publish(marker_array);
      wall_publisher->publish(arena_marker_array);
    }
  }

  bool obstacle_collision(turtlelib::RobotConfig robot_position){
    // Loop through each of the sensed obstacles in the marker array
    for(int i=0; i < (int)marker_array.markers.size(); i++){
        // Check if the obstacle intersects with the collision radius of the robot

        // Caclulate the straight line distance between center of robot to center of obstacle
        double dist_btw_centers = std::sqrt(
            (robot_position.x-marker_array.markers.at(i).pose.position.x)*(robot_position.x-marker_array.markers.at(i).pose.position.x)
            + (robot_position.y-marker_array.markers.at(i).pose.position.y)*(robot_position.y-marker_array.markers.at(i).pose.position.y));

        // If the distance between the robot center and obstacle center is less that the collision
        // radius + the obstacle radius, you are colliding
        if (dist_btw_centers <= collision_radius + obstacles_r){
          return true;
        }
    }
  return false;
  }

  bool wall_collision(turtlelib::RobotConfig robot_position){
    // If the robot's x or y position is greater than or equal to half of the length of the arena walls
    // minus half the width of the wall then you are colliding
    // Half because origin at 0,0
    if ((abs(robot_position.x)+collision_radius >= arena_x_len/2-0.2/2) || (abs(robot_position.y)+collision_radius >= arena_y_len/2-0.2/2)){
    return true;
    } else {return false;}
  }

  void wheel_commands_cb(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    // convert wheel command ticks to rad/s
    phi_l_rad_s = static_cast<double>(msg.left_velocity) * motor_cmd_per_rad_sec;
    phi_r_rad_s = static_cast<double>(msg.right_velocity) * motor_cmd_per_rad_sec;

    // Apply noise if there is any noise to apply and we are not sending stop commands
    if(input_noise > 0.0 && !turtlelib::almost_equal(msg.left_velocity, 0.0) && !turtlelib::almost_equal(msg.right_velocity, 0.0)){
      // Create a zero mean Gaussian distribution with a variance equal to the input nosie
      std::normal_distribution<> gaus_dist(0.0, input_noise);

      // Update the motor velocities to include the noise
      phi_r_rad_s += gaus_dist(get_random());
      phi_l_rad_s += gaus_dist(get_random());
    }

  }

  /// @brief SAY THAT I GOT THIS FROM MATT'S NOTES
  /// @return 
  std::mt19937 & get_random()
  {
      // static variables inside a function are created once and persist for the remainder of the program
      static std::random_device rd{}; 
      static std::mt19937 mt{rd()};
      // we return a reference to the pseudo-random number genrator object. This is always the
      // same object every time get_random is called
      return mt;
  }

  geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double theta){
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

  size_t count_;
  turtlelib::DiffDrive turtlebot = {1.0, 1.0};
  double wheel_radius = 0.0, track_width = 0.0, motor_cmd_per_rad_sec = 0.0; 
  double encoder_ticks_per_rad = 0.0, collision_radius = 0.0, phi_r_rad_s = 0.0, phi_l_rad_s = 0.0;
  double dt_time = 0.0, arena_x_len = 5.0, arena_y_len = 5.0, wall_height = 0.25;
  int motor_cmd_max = 0;
  double delta_wheel_pos_r = 0.0;
  double delta_wheel_pos_l = 0.0;
  double input_noise = 0.0, slip_fraction = 0.0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_commands_sub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;

  nuturtlebot_msgs::msg::SensorData sensor_readings;
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::MarkerArray arena_marker_array;
  nav_msgs::msg::Path visited_path;
  turtlelib::RobotConfig current_pos;
  turtlelib::WheelPos prev_wheel_pos = {0.0, 0.0};

    bool draw_only = false;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr heading_pub_;
    geometry_msgs::msg::Point heading;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NusimNode>());
  rclcpp::shutdown();
  return 0;
}
