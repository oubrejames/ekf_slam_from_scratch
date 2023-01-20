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

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class MinimalPublisher : public rclcpp::Node 
{
  public:
    MinimalPublisher()
    : Node("nusim"), count_(0)
    {
      time_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

      timer_ = this->create_wall_timer(
      5ms, std::bind(&MinimalPublisher::timer_callback, this));

      reset_server_ = this->create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&MinimalPublisher::reset, this, std::placeholders::_1, std::placeholders::_2));

      tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      teleport_server_ = this->create_service<nusim::srv::Teleport>(
        "~/teleport",
        std::bind(&MinimalPublisher::teleport, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    uint64_t timestep = 0;
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;
    double turtle_x=0.0, turtle_y=0.0, turtle_theta=0.0;
    

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
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
