#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <std_srvs/srv/empty.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class MinimalPublisher : public rclcpp::Node 
{
  public:
    MinimalPublisher()
    : Node("nusim"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      timer_ = this->create_wall_timer(
      5ms, std::bind(&MinimalPublisher::timer_callback, this));
      server_ = this->create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&MinimalPublisher::reset, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    uint64_t timestep = 0;

    void reset(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
              const std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
      (void)req;
      (void)res;
      timestep = 0;
    }
    
    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      message.data = timestep++;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
