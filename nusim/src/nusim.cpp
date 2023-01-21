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

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class NusimNode : public rclcpp::Node 
{
  public:
    NusimNode()
    : Node("nusim"), count_(0)
    {
      time_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

      auto hz_desc = rcl_interfaces::msg::ParameterDescriptor{};
      hz_desc.description = "Frequency of the  timer in Hz";
      this->declare_parameter("Hz", 200.0, hz_desc);
      int hz = this->get_parameter("Hz").get_parameter_value().get<double>();
      auto hz_in_ms = std::chrono::milliseconds((long)(1000/(hz)));
      timer_ = this->create_wall_timer(
      hz_in_ms, std::bind(&NusimNode::timer_callback, this));

      reset_server_ = this->create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&NusimNode::reset, this, std::placeholders::_1, std::placeholders::_2));

      tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      teleport_server_ = this->create_service<nusim::srv::Teleport>(
        "~/teleport",
        std::bind(&NusimNode::teleport, this, std::placeholders::_1, std::placeholders::_2));

      auto x0_desc = rcl_interfaces::msg::ParameterDescriptor();
      x0_desc.description = "Initial x position of turtlebot";
      this->declare_parameter("x0", 0.3, x0_desc);
      turtle_x0 = this->get_parameter("x0").get_parameter_value().get<double>();
      turtle_x=turtle_x0;

      auto y0_desc = rcl_interfaces::msg::ParameterDescriptor();
      y0_desc.description = "Initial y position of turtlebot";
      this->declare_parameter("y0", 0.0, y0_desc);
      turtle_y0 = this->get_parameter("y0").get_parameter_value().get<double>();
      turtle_y=turtle_y0;

      auto theta0_desc = rcl_interfaces::msg::ParameterDescriptor();
      theta0_desc.description = "Initial theta position of turtlebot";
      this->declare_parameter("theta0", 0.0, theta0_desc);
      turtle_theta0 = this->get_parameter("theta0").get_parameter_value().get<double>();
      turtle_theta=turtle_theta0;

      auto ob_x_desc = rcl_interfaces::msg::ParameterDescriptor();
      ob_x_desc.description = "List of x coordinates of obstacles";
      this->declare_parameter("obstacles/x", std::vector<double> {0.0, 0.5, 1.0}, ob_x_desc);
      obstacles_x = this->get_parameter("obstacles/x").get_parameter_value().get<std::vector<double>>();


      auto ob_y_desc = rcl_interfaces::msg::ParameterDescriptor();
      ob_y_desc.description = "List of y coordinates of obstacles";
      this->declare_parameter("obstacles/y", std::vector<double> {0.0, 0.5, 1.0}, ob_y_desc);
      obstacles_y = this->get_parameter("obstacles/y").get_parameter_value().get<std::vector<double>>();

      auto ob_r_desc = rcl_interfaces::msg::ParameterDescriptor();
      ob_r_desc.description = "Radius of cyindrical of obstacles";
      this->declare_parameter("obstacles/r", 0.1, ob_r_desc);
      obstacles_r = this->get_parameter("obstacles/r").get_parameter_value().get<double>();

      marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
      marker_array = make_marker_array();

    }

  private:
    uint64_t timestep = 0;
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;
    double turtle_x0, turtle_y0, turtle_theta0;
    double turtle_x, turtle_y, turtle_theta;
    double obstacles_r;
    std::vector<double> obstacles_x, obstacles_y;

    visualization_msgs::msg::MarkerArray make_marker_array(){
      visualization_msgs::msg::MarkerArray mkr_array;
      for(int i = 0; i < 2; i++){
        RCLCPP_INFO(this->get_logger(), "MAKING MARKER ARRAY: ");
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "nusim/world";
        marker.header.stamp = this->get_clock()->now();
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = obstacles_x[i];
        marker.pose.position.y = obstacles_y[i];
        marker.pose.position.z = 0.25/2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstacles_r*2;
        marker.scale.y = obstacles_r*2;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        mkr_array.markers.push_back(marker);
      }
      return mkr_array;
    }

    void teleport(const std::shared_ptr<nusim::srv::Teleport::Request> req,
                  std::shared_ptr<nusim::srv::Teleport::Response>){
      turtle_x = req->x;
      turtle_y = req->y;
      turtle_theta = req->theta;
    }

    void reset(const std::shared_ptr<std_srvs::srv::Empty::Request>,
              const std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      timestep = 0;
      turtle_x =turtle_x0;
      turtle_y = turtle_y0;
      turtle_theta = turtle_theta0;
    }
    
    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      message.data = timestep++;
      time_publisher_->publish(message);

      // Read message content and assign it to
      // corresponding tf variables
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "nusim/world";
      t.child_frame_id = "red/base_footprint";

      // Turtle only exists in 2D, thus we get x and y translation
      // coordinates from the message and set the z coordinate to 0
      t.transform.translation.x =turtle_x;
      t.transform.translation.y = turtle_y;
      q.setRPY(0, 0, turtle_theta);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
      marker_publisher_->publish( marker_array );

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;
    /// @brief //////////////////////////////////////////////////////////////
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NusimNode>());
  rclcpp::shutdown();
  return 0;
}
