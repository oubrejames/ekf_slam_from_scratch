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
    this->declare_parameter("wheel_radius", -1.0, wheel_radius_desc);
    wheel_radius = this->get_parameter("wheel_radius").get_parameter_value().get<double>();
    if(wheel_radius == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "Wheel radius not defined"); rclcpp::shutdown();}

    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor();
    track_width_desc.description = "Trackwidth between turtlebot wheels";
    this->declare_parameter("track_width", -1.0, track_width_desc);
    track_width = this->get_parameter("track_width").get_parameter_value().get<double>();
    if(track_width == -1.0){RCLCPP_ERROR_STREAM(this->get_logger(), "Track width not defined"); rclcpp::shutdown();}

    auto motor_cmd_max_desc = rcl_interfaces::msg::ParameterDescriptor();
    motor_cmd_max_desc.description = "Maximium wheel velocity command in ticks/sec";
    this->declare_parameter("motor_cmd_max", -1, motor_cmd_max_desc);
    motor_cmd_max = this->get_parameter("motor_cmd_max").get_parameter_value().get<int>();
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

    turtlebot = turtlelib::DiffDrive{track_width, wheel_radius};

    // Publisher to publish current timestep
    time_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Create frequency parameter, convert it to chrono ms for timer, and create timer
    auto hz_desc = rcl_interfaces::msg::ParameterDescriptor{};
    hz_desc.description = "Frequency of the  timer in Hz";
    this->declare_parameter("Hz", 200.0, hz_desc);
    int hz = this->get_parameter("Hz").get_parameter_value().get<double>();
    auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (hz)));
    timer_ = this->create_wall_timer(
      hz_in_ms, std::bind(&NusimNode::timer_callback, this));

    dt_time = 1.0/(double)hz;

    // Define reset server
    reset_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&NusimNode::reset, this, std::placeholders::_1, std::placeholders::_2));

    // Define transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Define teleport service
    teleport_server_ = this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&NusimNode::teleport, this, std::placeholders::_1, std::placeholders::_2));

    // Create parameters for turtlebot position
    auto x0_desc = rcl_interfaces::msg::ParameterDescriptor();
    x0_desc.description = "Initial x position of turtlebot";
    this->declare_parameter("x0", 0.3, x0_desc);
    turtle_x0 = this->get_parameter("x0").get_parameter_value().get<double>();
    turtle_x = turtle_x0;

    auto y0_desc = rcl_interfaces::msg::ParameterDescriptor();
    y0_desc.description = "Initial y position of turtlebot";
    this->declare_parameter("y0", 0.0, y0_desc);
    turtle_y0 = this->get_parameter("y0").get_parameter_value().get<double>();
    turtle_y = turtle_y0;

    auto theta0_desc = rcl_interfaces::msg::ParameterDescriptor();
    theta0_desc.description = "Initial theta position of turtlebot";
    this->declare_parameter("theta0", 0.0, theta0_desc);
    turtle_theta0 = this->get_parameter("theta0").get_parameter_value().get<double>();
    turtle_theta = turtle_theta0;

    // Create parameters for obstacle position and size
    auto ob_x_desc = rcl_interfaces::msg::ParameterDescriptor();
    ob_x_desc.description = "List of x coordinates of obstacles";
    this->declare_parameter("obstacles/x", std::vector<double> {}, ob_x_desc);
    obstacles_x =
      this->get_parameter("obstacles/x").get_parameter_value().get<std::vector<double>>();

    auto ob_y_desc = rcl_interfaces::msg::ParameterDescriptor();
    ob_y_desc.description = "List of y coordinates of obstacles";
    this->declare_parameter("obstacles/y", std::vector<double> {}, ob_y_desc);
    obstacles_y =
      this->get_parameter("obstacles/y").get_parameter_value().get<std::vector<double>>();

    auto ob_r_desc = rcl_interfaces::msg::ParameterDescriptor();
    ob_r_desc.description = "Radius of cyindrical of obstacles";
    this->declare_parameter("obstacles/r", 0.05, ob_r_desc);
    obstacles_r = this->get_parameter("obstacles/r").get_parameter_value().get<double>();

    // Define a publisher to publish obstacle markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      10);

    // Only create marker array if there are actually obstacles defines
    if (obstacles_x.size()) {
      marker_array = make_marker_array();
    }

    // Report error and kill node if obstacle x and y arrays are not same length
    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "obstacles_x and obstacles_y lists have different lengths, shutting down.");
      rclcpp::shutdown();
      return;
    }


    // Create wheel command subscriber
    wheel_commands_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&NusimNode::wheel_commands_cb, this, std::placeholders::_1));


    // Create publisher to publish sensor data
    sensor_data_pub_ = this->create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

  }

private:
  uint64_t timestep = 0;
  geometry_msgs::msg::TransformStamped t;
  tf2::Quaternion q;
  double turtle_x0, turtle_y0, turtle_theta0;
  double turtle_x, turtle_y, turtle_theta;
  double obstacles_r;
  std::vector<double> obstacles_x, obstacles_y;

  /// @brief create an array of markers based of parameter input (obstacles/x, obstacles/y, obstacles/r)
  /// @return marker array with all obstacles
  visualization_msgs::msg::MarkerArray make_marker_array()
  {
    visualization_msgs::msg::MarkerArray mkr_array;
    for (int i = 0; i < (int)(obstacles_x.size()); i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = this->get_clock()->now();
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacles_x[i];
      marker.pose.position.y = obstacles_y[i];
      marker.pose.position.z = 0.25 / 2;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacles_r * 2;
      marker.scale.y = obstacles_r * 2;
      marker.scale.z = 0.25;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      mkr_array.markers.push_back(marker);
    }
    return mkr_array;
  }

  /// @brief Teleport the turtlebot from one location to another
  /// @param req Custom srv containing a x,y,theta position
  /// @param - service response (empty)
  void teleport(
    const std::shared_ptr<nusim::srv::Teleport::Request> req,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
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
    auto message = std_msgs::msg::UInt64();
    message.data = timestep++;
    time_publisher_->publish(message);

    // Update world to robot transformation info
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = turtle_x;
    t.transform.translation.y = turtle_y;
    q.setRPY(0.0, 0.0, turtle_theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send transformation
    tf_broadcaster_->sendTransform(t);
    marker_publisher_->publish(marker_array);

    // Update current config of robot based on wheel commands
    turtlelib::WheelPos cur_wheel_pos = turtlebot.get_current_wheel_pos();
    double new_wheel_pos_r =(dt_time*phi_r_rad_s);
    double new_wheel_pos_l = (dt_time*phi_l_rad_s);
    turtlebot.forward_kinematics({new_wheel_pos_r, new_wheel_pos_l});

    current_pos = turtlebot.get_current_pos();

    // Update turtlebot position for broadcasting
    turtle_x = current_pos.x;
    turtle_y = current_pos.y;
    turtle_theta = current_pos.theta;

    // update sensor data 
    sensor_readings.stamp = this->get_clock()->now();
    sensor_readings.left_encoder = (prev_wheel_pos.l+new_wheel_pos_l)*encoder_ticks_per_rad;
    sensor_readings.right_encoder = (prev_wheel_pos.r+new_wheel_pos_r)*encoder_ticks_per_rad;
    prev_wheel_pos = turtlebot.get_current_wheel_pos();
    // publish sensor data
    sensor_data_pub_->publish(sensor_readings);
  }

  void wheel_commands_cb(const nuturtlebot_msgs::msg::WheelCommands & msg){
    // convert wheel command ticks to rad/s
    phi_l_rad_s = msg.left_velocity*motor_cmd_per_rad_sec;
    phi_r_rad_s = msg.right_velocity*motor_cmd_per_rad_sec;


  }

  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;
  visualization_msgs::msg::MarkerArray marker_array;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  // you are here, creating wheel command sub
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_commands_sub_;
  turtlelib::DiffDrive turtlebot = {1.0, 1.0};
  double wheel_radius, track_width, motor_cmd_per_rad_sec;
  int motor_cmd_max;
  double encoder_ticks_per_rad, collision_radius;
  double phi_r_rad_s=0.0, phi_l_rad_s=0.0;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  nuturtlebot_msgs::msg::SensorData sensor_readings;
  turtlelib::RobotConfig current_pos;
  turtlelib::WheelPos prev_wheel_pos = {0.0, 0.0};
  double dt_time;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NusimNode>());
  rclcpp::shutdown();
  return 0;
}
