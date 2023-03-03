/// \file
/// \brief This file contains the a node that simulates the lidar of the Turtlebot
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
///     \param angle_min (double): Minumum angle on laser scan
///     \param angle_max (double): Maximum angle on laser scan
///     \param angle_increment (double): The angle that the scanner is incrimenting by
///     \param max_range (double): Maximum range for fake laser scanner (m)
///     \param min_range (double): Minimum range for fake laser scanner (m)
///     \param noise_fake_laser (double): Variance for a zero mean noise applied to the fake laser scanner
///     \param arena_x_len (double): Length of arena in the x direction
///     \param arena_y_len (double): Length of arena in the y direction
///
/// PUBLISHES:
///     topic_name (topic_type): description of topic
///     /fake_laser_scan (sensor_msgs::msg::LaserScan): Publishes the fake sensor measurements for the Turtlebot
///
/// SUBSCRIBES:
///     topic_name (topic_type): description of topic
///     /nusim/obstacles (visualization_msgs::msg::MarkerArray): Subscribes to markers representing the obstacles
///     /heading (visualization_msgs::msg::MarkerArray): Subscribes to Turtlebot's heading

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <random>
#include "tf2_ros/transform_listener.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "tf2_ros/buffer.h"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <tuple>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "turtlelib/rigid2d.hpp"
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;

class FakeLaser : public rclcpp::Node
{
public:
    FakeLaser()
    : Node("fake_laser_scan"), count_(0)
    {

        // Define parameter for minumum angle on laser scan
        auto angle_min_desc = rcl_interfaces::msg::ParameterDescriptor();
        angle_min_desc.description = "Minumum angle on laser scan";
        declare_parameter("angle_min", 0.0, angle_min_desc);
        angle_min = get_parameter("angle_min").get_parameter_value().get<double>();

        // Define parameter for maximum angle on laser scan
        auto angle_max_desc = rcl_interfaces::msg::ParameterDescriptor();
        angle_max_desc.description = "Maximum angle on laser scan";
        declare_parameter("angle_max", 6.2657318115234375, angle_max_desc);
        angle_max = get_parameter("angle_max").get_parameter_value().get<double>();

       // Define parameter for angle incriment on laser scan
        auto angle_increment_desc = rcl_interfaces::msg::ParameterDescriptor();
        angle_increment_desc.description = "The angle that the scanner is incrimenting by";
        declare_parameter("angle_increment", 0.01745329238474369, angle_increment_desc);
        angle_increment = get_parameter("angle_increment").get_parameter_value().get<double>();

        // Define parameter to change maximum range of the sensor
        auto max_range_desc = rcl_interfaces::msg::ParameterDescriptor();
        max_range_desc.description = "Maximum range for fake laser scanner (m)";
        declare_parameter("max_range", 3.5, max_range_desc);
        max_range = get_parameter("max_range").get_parameter_value().get<double>();

        // Define parameter to change maximum range of the sensor
        auto min_range_desc = rcl_interfaces::msg::ParameterDescriptor();
        min_range_desc.description = "Minimum range for fake laser scanner (m)";
        declare_parameter("min_range", 0.11999999731779099, min_range_desc);
        min_range = get_parameter("min_range").get_parameter_value().get<double>();

        // Define parameter to change noise of the sensor
        auto noise_fake_laser_desc = rcl_interfaces::msg::ParameterDescriptor();
        noise_fake_laser_desc.description = "Variance for a zero mean noise applied to the fake laser scanner";
        declare_parameter("noise_fake_laser", 0.01, noise_fake_laser_desc);
        noise_fake_laser = get_parameter("noise_fake_laser").get_parameter_value().get<double>();

        // Define arena wall parameters
        auto arena_x_len_desc = rcl_interfaces::msg::ParameterDescriptor();
        arena_x_len_desc.description = "Length of arena in the x direction";
        declare_parameter("~x_length", 5.0, arena_x_len_desc);
        arena_x_len = get_parameter("~x_length").get_parameter_value().get<double>();

        auto arena_y_len_desc = rcl_interfaces::msg::ParameterDescriptor();
        arena_y_len_desc.description = "Length of arena in the y direction";
        declare_parameter("~y_length", 5.0, arena_y_len_desc);
        arena_y_len = get_parameter("~y_length").get_parameter_value().get<double>();

        // Create 5 Hz timer
        auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (5.0)));
        timer_ = create_wall_timer(
        hz_in_ms, std::bind(&FakeLaser::timer_callback, this));

        // Create listener to get laser scanner location
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create subcriber to get actual obstacle positions
         real_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/nusim/obstacles", 10, std::bind(&FakeLaser::real_obstacles_cb, this, std::placeholders::_1));

        // Publisher to publish sensed obstacles
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("fake_laser_scan", 10);

        // publisher to get heading
         heading_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "heading", 10, std::bind(&FakeLaser::heading_cb, this, std::placeholders::_1));
    }

private:

    // Timer callback going at 5 hz
    void timer_callback()
    {
        do_a_laser_scan();
    }

    void do_a_laser_scan(){
        // Create a laser scan message
        laser_scan.header.stamp = this->get_clock()->now();
        laser_scan.header.frame_id = "red/base_scan";
        laser_scan.angle_min = angle_min;
        laser_scan.angle_max = angle_max;
        laser_scan.angle_increment = angle_increment;
        // laser_scan.scan_time = 0.20066890120506287;
        // laser_scan.time_increment = 0.0005574136157520115;
        laser_scan.range_min = min_range;
        laser_scan.range_max = max_range;

        laser_scan.ranges.clear();

        // Check position of the robot 
        // Look up for the transformation between nusim/world and the red robot laser frames
        geometry_msgs::msg::TransformStamped laser_tf;
        try {
          laser_tf = tf_buffer_->lookupTransform(
            "nusim/world", "red/base_scan",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            get_logger(), "Could not transform red/base_scan to nusim/world");
          return;
        }

        // Loop through each angle that the laser scans at to get individual readings
        for(double i = angle_min; i < angle_max; i += angle_increment){

            // add robot angle
            tf2::Quaternion q(laser_tf.transform.rotation.x,  laser_tf.transform.rotation.y,  laser_tf.transform.rotation.z,  laser_tf.transform.rotation.w);

            auto const theta = heading;// turtlelib::normalize_angle(q.getAngle());
            auto const x_max_range = max_range*std::cos(i+theta) + laser_tf.transform.translation.x;
            auto const y_max_range = max_range*std::sin(i+theta) + laser_tf.transform.translation.y;

            double range = 0.0, range_to_push = 0.0;

            // Loop through each of the obstacles looking for intersections
            // Create an array to check if one lidar scan intersects with multiple objects and pick the right one
            std::vector<double> scans_to_check(obstacle_array.markers.size());

            // Loop through each of the obstacles looking for intersections
            for (size_t j = 0; j < obstacle_array.markers.size(); j++){
                range = check_laser_intersect({laser_tf.transform.translation.x, laser_tf.transform.translation.y}, {x_max_range, y_max_range}, {obstacle_array.markers.at(j).pose.position.x, obstacle_array.markers.at(j).pose.position.y}, 0.038);
                scans_to_check.at(j) = range;
            }

            // Sort array of intersection ranges
            std::sort(scans_to_check.begin(), scans_to_check.end());

            // Push back closest obstacle
            for(size_t n = 0; n < scans_to_check.size(); n++){
                if(scans_to_check.at(n) > 0.0){
                    range_to_push = scans_to_check.at(n);
                    break;
                }
            }
            laser_scan.ranges.push_back(range_to_push);
        }
        laser_scan_pub_->publish(laser_scan);
    }

    double check_wall_intersect(std::tuple<double, double> laser, std::tuple<double, double> max){

            // Calculate slope
            auto m = (std::get<1>(max) - std::get<1>(laser))/(std::get<0>(max) - std::get<0>(laser));

            // Define arena bounds
            auto const y_top_arena = (arena_y_len/2.0 - 0.1);
            auto const y_bot_arena = -(arena_y_len/2.0 - 0.1);
            auto const x_left_arena = -(arena_x_len/2.0 - 0.1);
            auto const x_right_arena = (arena_x_len/2.0 - 0.1);

            // Calculate the corresponding points 
            auto const x_top = (y_top_arena-std::get<1>(laser))/m + std::get<0>(laser);
            auto const x_bot = (y_bot_arena-std::get<1>(laser))/m + std::get<0>(laser);
            auto const y_left = m*(x_left_arena-std::get<0>(laser))+std::get<1>(laser);
            auto const y_right = m*(x_right_arena-std::get<0>(laser))+std::get<1>(laser);

            // Calculate the distance to each wall
            auto const d_left = std::sqrt((x_left_arena-std::get<0>(laser))*(x_left_arena-std::get<0>(laser)) + (y_left-std::get<1>(laser))*(y_left-std::get<1>(laser)));
            auto const d_right = std::sqrt((x_right_arena-std::get<0>(laser))*(x_right_arena-std::get<0>(laser)) + (y_right-std::get<1>(laser))*(y_right-std::get<1>(laser)));
            auto const d_top = std::sqrt((x_top-std::get<0>(laser))*(x_top-std::get<0>(laser)) + (y_top_arena-std::get<1>(laser))*(y_top_arena-std::get<1>(laser)));
            auto const d_bot = std::sqrt((x_bot-std::get<0>(laser))*(x_bot-std::get<0>(laser)) + (y_bot_arena-std::get<1>(laser))*(y_bot_arena-std::get<1>(laser)));

            // Put distances into an array and get the smallest one
            std::vector<double> distances = {d_left, d_right, d_top, d_bot};
            std::vector<double> x_points = {x_left_arena, x_right_arena, x_top, x_bot};
            std::vector<double> y_points = {y_left, y_right, y_top_arena, y_bot_arena};

            // Loop through all the distances and return the smallest one that is valid
            // If the smallest is invalid, pop it
            bool flag = true;
            while(flag){
            auto min_idx = get_min_idx(distances);

                if (((x_points.at(min_idx) - std::get<0>(laser))/(std::get<0>(max) - std::get<0>(laser))) > 0.0){
                    flag = false;
                    return distances.at(min_idx);
                }
                else{
                    distances.erase (distances.begin() + min_idx);
                    x_points.erase (x_points.begin() + min_idx);
                    y_points.erase (y_points.begin() + min_idx);
                }
            }
            return 0.0;
    }

    size_t get_min_idx(std::vector<double> arr){
        auto min_idx = 0;
        for(size_t i = 0; i < arr.size(); i++){
            if(arr.at(i)<arr.at(min_idx)){
                min_idx = i;
            }
        }
        return min_idx;
    }

    double check_laser_intersect(std::tuple<double, double> laser, std::tuple<double, double> max, std::tuple<double, double> obstacle, double obs_radius){
        // Calculate possible x and y componenets for both plus and minus
        auto const m = (std::get<1>(max)-std::get<1>(laser))/(std::get<0>(max)-std::get<0>(laser));

        auto const a = 1 + m*m;

        auto const alpha = std::get<1>(laser) - m*std::get<0>(laser)-std::get<1>(obstacle);
        auto const b = 2*(alpha*m-std::get<0>(obstacle));
        auto const c = std::get<0>(obstacle)*std::get<0>(obstacle) + alpha*alpha - obs_radius*obs_radius;

        double range = 0.0;

        // Calculate discriminant
        double disc = b*b - 4*a*c;

        if( disc > 0){

            auto const xp = (-b + std::sqrt(disc))/(2*a);
            auto const xm = (-b - std::sqrt(disc))/(2*a);

            auto const yp = m * (xp - std::get<0>(laser)) + std::get<1>(laser);
            auto const ym = m * (xm - std::get<0>(laser)) + std::get<1>(laser);

            // Get Euclidean distance to each point
            auto const dp = std::sqrt((xp-std::get<0>(laser))*(xp-std::get<0>(laser))
                                       +(yp-std::get<1>(laser))*(yp-std::get<1>(laser)));

            auto const dm = std::sqrt((xm-std::get<0>(laser))*(xm-std::get<0>(laser))
                                       +(ym-std::get<1>(laser))*(ym-std::get<1>(laser)));

            auto x = 0.0, y = 0.0;

            // Set range to point with shortest distance
            if (dp < dm){
                range = dp;
                x = xp;
                y=yp;
            }
            else{
                range = dm;
                x = xm;
                y = ym;
                }

        // Check if reading is within range and the heading of point compared to obstacle
        if((range < min_range) || (range > max_range) || ((x - std::get<0>(laser)) / (std::get<0>(max) - std::get<0>(laser)) < 0.0)|| ( (y - std::get<1>(laser)) / (std::get<1>(max) - std::get<1>(laser)) < 0.0)){
            range = 0.0;
        }
        }
    
    // If not intersection with obstacles check walls
    else{
        range = check_wall_intersect(laser, max);
    }
        // Get a Gaussian distributin of noise
        std::normal_distribution<> gaus_dist(0.0, noise_fake_laser);
        range += gaus_dist(get_random());
        return range;
    }

    // This function was copied from
    // https://nu-msr.github.io/navigation_site/lectures/gaussian.html
    std::mt19937 & get_random()
    {
        static std::random_device rd{}; 
        static std::mt19937 mt{rd()};
        return mt;
    }

    // Subscribe to get the actual marker positions
    void real_obstacles_cb(const visualization_msgs::msg::MarkerArray & msg){
        obstacle_array = msg;
    }


    void heading_cb(const geometry_msgs::msg::Point & msg){
        heading = msg.z;
    }

    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr real_obstacles_sub_;
    visualization_msgs::msg::MarkerArray obstacle_array;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double angle_min = 0.0, angle_max = 0.0, angle_increment = 0.0, min_range = 0.0, max_range = 0.0;

    sensor_msgs::msg::LaserScan laser_scan;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

    double arena_x_len = 5.0;
    double arena_y_len = 5.0;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr heading_sub_;
    double heading = 0.0, noise_fake_laser = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLaser>());
    rclcpp::shutdown();
    return 0;
}
