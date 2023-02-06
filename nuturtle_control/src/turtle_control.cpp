#include "rclcpp/rclcpp.hpp"
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

    // Create DiffDrive object 
    turtlebot = turtlelib::DiffDrive{track_width, wheel_radius};

    // Create cmd_vel subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::velocity_cb, this, std::placeholders::_1));

    // Create publisher to publish wheel commands
    wheel_cmd_pub_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_commands", 10);

    // Create sensor_data subscriber
    sensor_data_sub_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_data_cb, this, std::placeholders::_1));

    // Create publisher to publish joint states
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Initialize turtle joint state
    turtle_joint_state.name = {"wheel_right_joint", "wheel_left_joint"};
    turtle_joint_state.position = {0.0 , 0.0};
    turtle_joint_state.velocity = {0.0 , 0.0};
  }

private:
    /// @brief subscribe to the cmd_vel topic to get robot velocity commands (rads/s)
    /// @param msg the Twist from the cmd_vel topic
    void velocity_cb(const geometry_msgs::msg::Twist & msg)
    { 
        velocity_command = msg;

        // Get command twist
        double w = msg.angular.z;
        double x = msg.linear.x;
        double y = msg.linear.y;
        turtlelib::Twist2D twst = {w,x,y};
        // RCLCPP_ERROR_STREAM(this->get_logger(), "msg.angular.z " << msg.angular.z);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "msg.linear.x " << msg.linear.x);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "msg.linear.y " << msg.linear.y);

        // Calculate wheel velocities from twist with IK (rad/s)
        turtlelib::WheelPos wheel_command_rad_s = turtlebot.inverse_kinematics(twst);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "WheelPos_r " << wheel_command_rad_s.r);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "WheelPos_l " << wheel_command_rad_s.l);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "============================================");

        // Convert to a wheel command msgs (ticks)
        nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
        wheel_cmd_msg.left_velocity = wheel_command_rad_s.l/motor_cmd_per_rad_sec;
        wheel_cmd_msg.right_velocity = wheel_command_rad_s.r/motor_cmd_per_rad_sec;

        // Check if wheel commands are over max velocity
        // Convert velocity from rads/sec to ticks/sec to compare to motor_cmd_max
        if(wheel_cmd_msg.left_velocity > motor_cmd_max){
            wheel_cmd_msg.left_velocity = motor_cmd_max;
        }
        if(wheel_cmd_msg.right_velocity > motor_cmd_max){
            wheel_cmd_msg.right_velocity = motor_cmd_max;
        }
        if(wheel_cmd_msg.right_velocity < -motor_cmd_max){
            wheel_cmd_msg.right_velocity = -motor_cmd_max;
        }
        if(wheel_cmd_msg.left_velocity < -motor_cmd_max){
            wheel_cmd_msg.left_velocity = -motor_cmd_max;
        }
        // Publish wheel commands in ticks
        wheel_cmd_pub_->publish(wheel_cmd_msg);
    }

    /// @brief subscribe to the sensor_data topic and publish updated joint states
    /// @param msg readings from wheel encoders
    void sensor_data_cb(const nuturtlebot_msgs::msg::SensorData & msg)
    {
        // if first iteration, make previous timestep 0
        if(init_flag){
            prev_timestep.header.stamp = this->get_clock()->now();
            init_flag = false;
        }

        // Get encoder values in ticks
        double l_encoder_ticks = msg.left_encoder;
        double r_encoder_ticks = msg.right_encoder;

        // Convert encoder ticks to radians (change in position)
        double l_encoder_rad = (l_encoder_ticks)/encoder_ticks_per_rad;
        double r_encoder_rad = (r_encoder_ticks)/encoder_ticks_per_rad;

        // Calculate change in time between sensor readings
        double dt = (msg.stamp.sec + msg.stamp.nanosec*1e-9) -
                        (prev_timestep.header.stamp.sec + prev_timestep.header.stamp.nanosec*1e-9);

        // RCLCPP_ERROR_STREAM(this->get_logger(), "msg.stamp.sec+ns " << msg.stamp.sec);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "prev_timestep" << prev_timestep.header.stamp.sec);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "dt" << dt);
        // RCLCPP_ERROR_STREAM(this->get_logger(), "l_encoder_rad" << l_encoder_rad);

        // Calculate wheel velocities (rad/s)
        double r_vel = (r_encoder_rad)/dt;
        double l_vel = (l_encoder_rad)/dt;

        // Update new wheel position in rad and velocity in rad/s 
        turtle_joint_state.header.stamp = this->get_clock()->now();
        turtle_joint_state.position[0] = r_encoder_rad;
        turtle_joint_state.position[1] = l_encoder_rad;
        turtle_joint_state.velocity[0] = r_vel;
        turtle_joint_state.velocity[1] = l_vel;

        // save previous pose
        prev_timestep.header.stamp = this->get_clock()->now();
        prev_timestep.name = {"right_wheel", "left_wheel"};
        prev_timestep.position = {r_encoder_rad, l_encoder_rad};
        prev_timestep.velocity = {r_vel, l_vel};

        // Publish joint states
        joint_state_pub_->publish(turtle_joint_state);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
    double wheel_radius, track_width, motor_cmd_per_rad_sec;
    int motor_cmd_max;
    double encoder_ticks_per_rad, collision_radius;
    size_t count_;
    geometry_msgs::msg::Twist velocity_command;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
    turtlelib::DiffDrive turtlebot = {1.0, 1.0};
    sensor_msgs::msg::JointState turtle_joint_state;
    bool init_flag = true;
    sensor_msgs::msg::JointState prev_timestep;
    double motor_cmd_max_rad_sec;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
