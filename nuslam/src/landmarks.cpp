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
#include <sensor_msgs/msg/laser_scan.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks_node"), count_(0)
  {
    // publisher to get laser scan data
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan >(
      "fake_laser_scan", 10, std::bind(&Landmarks::laser_scan_cb, this, std::placeholders::_1));

    // Create listener to get laser scanner location
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cluster_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "clusters", 10);

  }

private:
  void timer_callback()
  {

  }


  auto get_robot_location()
  {
    // Check position of the robot
    // Look up for the transformation between nusim/world and the green robot
    geometry_msgs::msg::TransformStamped laser_tf;
    try {
      laser_tf = tf_buffer_->lookupTransform(
        "nusim/world", "green/base_footprint",
        tf2::TimePointZero);
      return laser_tf;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        get_logger(), "Could not transform red/base_scan to nusim/world");
      return laser_tf;
    }
  } 


  auto basic_laser_cluster(const sensor_msgs::msg::LaserScan & msg)
  {
    auto tolerance = 0.05; // 5 cm

    // Clusters are represented by a single point, the centerpoint of the cluster
    std::vector<std::vector<geometry_msgs::msg::Point>> clusters;

    // Current cluster is all the points within the threshold. Will be processed for circle regression
    std::vector<geometry_msgs::msg::Point> current_cluster;

    auto robot_tf = get_robot_location();
    double xx = static_cast<double>(robot_tf.transform.translation.x);
    double yy = static_cast<double>(robot_tf.transform.translation.y);
    turtlelib::Vector2D turtle_pos ={xx,yy};

    auto current_angle = msg.angle_min + msg.angle_increment;

    turtlelib::PolarVector2D first_reading{msg.ranges.at(0), current_angle};

    // I think current issue is something to do with not taking rotation into account
    // project_from_coord is just giving me the x,y offset relative from my straight bearing
    


    // TODO convert first point to cartesian - project to coord still needs to take 
    // robot location into account, right now it just does relative x, y
    // current_cluster.push_back(first_reading.project_from_coord(turtle_pos));

    for(int idx = 0; idx < msg.ranges.size()-1; idx+=1)
    {
      auto current_range = msg.ranges.at(idx);
      auto last_adjascent_range = msg.ranges.at(idx+1);

      // Cluster if the ranges within some tolerance
      if (abs(current_range - last_adjascent_range) < tolerance){
        // Add to cluster // TODO could be more efficient by not creating new instance every time and overriding values
        turtlelib::PolarVector2D point_to_add{msg.ranges.at(idx), msg.angle_min+msg.angle_increment*idx};
        
        auto vec_to_cluster = point_to_add.project_from_coord(turtle_pos);
        geometry_msgs::msg::Point point_in_robot_frame;
        point_in_robot_frame.x = vec_to_cluster.x;
        point_in_robot_frame.y = vec_to_cluster.y;
        point_in_robot_frame.z = 0.0;


        // geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));

        geometry_msgs::msg::Point point_in_world_frame;

        tf2::doTransform(point_in_robot_frame, point_in_world_frame, robot_tf);


        current_cluster.push_back(point_in_world_frame);
      }
      else{
        // Close that cluster and make another
        if (current_cluster.size() > 2){
          clusters.push_back(current_cluster);
          current_cluster.clear();
        }
      }
    }

    return clusters;
  }

  auto cluster_to_marker(const std::vector<std::vector<turtlelib::Vector2D>> & clusters){
    visualization_msgs::msg::MarkerArray cluster_marker_array;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    for(int i = 0; i < clusters.size(); i ++ ){
      // TODO actual center point
      turtlelib::Vector2D center_point = {clusters.at(i).at(0).x,clusters.at(i).at(0).y};
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;
      marker.pose.position.x = center_point.x;
      marker.pose.position.y = center_point.y;
      marker.pose.position.z = 0.25 / 2;

      // marker.pose.orientation.x = q.x();
      // marker.pose.orientation.y = q.y();
      // marker.pose.orientation.z = q.z();
      // marker.pose.orientation.w = q.w();
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      cluster_marker_array.markers.push_back(marker);
    }
    return cluster_marker_array;
  }

  void laser_scan_cb(const sensor_msgs::msg::LaserScan & msg)
  {
    // Start with some kind of basic clustering
    auto clusters = basic_laser_cluster(msg);

    // Convert clusters to markers and publish markers
    cluster_vis_pub_->publish(cluster_to_marker(clusters));

  }


  size_t count_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_vis_pub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
