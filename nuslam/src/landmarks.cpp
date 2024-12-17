/// \file
/// TODO: Document this like the other files


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

class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks_node"), count_(0)
  {
    // publisher to get laser scan data
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan >(
      "fake_laser_scan", 10, std::bind(&FakeLaser::laser_scan_cb, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {

  }

  void laser_scan_cb(const sensor_msgs::msg::LaserScan & msg)
  {
    // Start with some kind of basic clustering
  }

  auto basic_laser_cluster(cont sensor_msgs::msg::LaserScan & msg)
  {
    auto tolerance = 0.05; // 5 cm

    // Clusters are represented by a single point, the centerpoint of the cluster
    std::vector<turtlelib::Vector2D> clusters;

    // Current cluster is all the points within the threshold. Will be processed for circle regression
    std::vector<turtlelib::Vector2D> current_cluster;
    auto current_angle = msg.angle_min + msg.angle_increment;
    turtlelib::PolarVector2D last_point{msg.ranges.at(0), current_angle};
    current_cluster.ranges.append(last_point.project_to_coord());

    for(int increment = 1; increment < msg.ranges.size(); increment++)
    {
      auto current_range = msg.ranges.at(increment);
      auto adjascent_range = msg.ranges.at(increment-1);

      // Cluster if the ranges within some tolerance
      if abs(current_range - adjascent_range){
        // Add to cluster
        current_cluster.ranges.append(current_cluster);
        current_cluster.ranges.append(current_cluster);

      }
    }

  }

  auto scan_to_cartesian() 

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNode>());
  rclcpp::shutdown();
  return 0;
}
