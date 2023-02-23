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

        // temp point pub to get heading
         heading_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "heading", 10, std::bind(&FakeLaser::heading_cb, this, std::placeholders::_1));
    }

private:

    // Timer callback going at 5 hz
    void timer_callback()
    {

        do_a_laser_scan();
        // // Get a Gaussian distributin of noise
        // std::normal_distribution<> gaus_dist(0.0, basic_sensor_variance);

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
        int count = 0.0;
        // Loop through each angle that the laser scans at to get individual readings
        for(double i = angle_min; i < angle_max; i += angle_increment){

            // add robot angle
            tf2::Quaternion q(laser_tf.transform.rotation.x,  laser_tf.transform.rotation.y,  laser_tf.transform.rotation.z,  laser_tf.transform.rotation.w);

            RCLCPP_ERROR_STREAM(get_logger(), "Angle " << q.getAngle()); 
            RCLCPP_ERROR_STREAM(get_logger(), "Normalized angle " << turtlelib::normalize_angle(q.getAngle())); 
            auto const theta = heading;// turtlelib::normalize_angle(q.getAngle());
            auto const x_max_range = max_range*std::cos(i+theta) + laser_tf.transform.translation.x;
            auto const y_max_range = max_range*std::sin(i+theta) + laser_tf.transform.translation.y;

            double range = -999.0;

            // Loop through each of the obstacles looking for intersections
            for (size_t j = 0; j < obstacle_array.markers.size(); j++){
                range = check_laser_intersect({laser_tf.transform.translation.x, laser_tf.transform.translation.y}, {x_max_range, y_max_range}, {obstacle_array.markers.at(j).pose.position.x, obstacle_array.markers.at(j).pose.position.y}, 0.038);

                count ++;
                if(range > 0.0){
                    // RCLCPP_ERROR_STREAM(get_logger(), "DETECTION " << range);
                    break;
                }
            }

            // If there is an intersection with this obstacle
            laser_scan.ranges.push_back(range);
            // // If no intersections with obstacles check wall
            // if (range == -999.0){
            //     auto wall_reading = check_wall_intersect({x_max_range, y_max_range});

            //     // If there is an intersection with a wall
            //     if (std::get<2>(wall_reading)){
            //         // Calculate range distance
            //         range = std::sqrt((std::get<0>(wall_reading)*std::get<0>(wall_reading)+std::get<1>(wall_reading)*std::get<1>(wall_reading)));
            //     }
            // }
            // Add ranges to laser scan message
            // laser_scan.ranges.push_back(range);

            // if(laser_scan.ranges.size() <1){
            // laser_scan.ranges.push_back(range);}
            // else{laser_scan.ranges.push_back(laser_scan.ranges.at(0));}
        }
        // Publish laser scan

        laser_scan_pub_->publish(laser_scan);

    }

    // double check_wall_intersect(std::tuple<double, double> laser, std::tuple<double, double> max){

    //         // Define points to return later
    //        double x = 0.0, y = 0.0, m = 0.0;

    //         // Check if intersecting with wall (checking if within bounds)

    //         // if the x is out of bounds update the x to return to be point on wall, else keep same
    //         if (abs(std::get<0>(max)) > arena_x_len/2.0 - 0.1){
    //             x = sgn(std::get<0>(max))*abs(((std::get<0>(laser))-arena_x_len/2.0 - 0.1));
                
    //             // Calculate slope from robot to max range point
    //             // m = (ymax - ylaser)/(xmax - xlaser)
    //             m = (std::get<1>(max) - std::get<1>(laser))/(std::get<0>(max) - std::get<0>(laser));

    //             // Calculate y value corresponding to the x value
    //             // y = m*(x-xmax)+ymax
    //             y = m * (x - std::get<0>(max)) + std::get<1>(max);
    //         }

    //         // if the y is out of bounds update the x to return to be point on wall
    //         if (abs(std::get<1>(max)) > arena_y_len/2.0 - 0.1){
    //             y = sgn(std::get<1>(max))*abs(((std::get<1>(laser))-arena_y_len/2.0 - 0.1));

    //             // Calculate slope from robot to max range point
    //             // m = (ymax - ylaser)/(xmax - xlaser)
    //             m = (std::get<1>(max) - std::get<1>(laser))/(std::get<0>(max) - std::get<0>(laser));

    //             // Calculate x value corresponding to the y value
    //             // x = (y - ymax)/m +xmax
    //             x = (y - std::get<1>(max))/m + std::get<0>(max);
    //         }
    //         RCLCPP_ERROR_STREAM(get_logger(), "X: " << x << "\n Y: " << y << "\n RANGE: " << std::sqrt(x*x+y*y)); 

    //         // Check if less than min range
    //         if(std::sqrt(x*x + y*y) < min_range){
    //             return {999, 999, false};
    //         }
    //         else{return {x, y, true};}
    // }

    /// @brief 
    /// @param laser tuple consisting of the laser's position x = laser[0], y = laser[1]
    /// @param max tuple consisting of the position at the laser's max range x = max[0], y = max[1]
    /// @param obstacle tuple consisting of an obstacles position x = obstacle[0], y = obstacle[1]
    /// @param obs_radius 
    /// @return 
    double check_laser_intersect(std::tuple<double, double> laser, std::tuple<double, double> max, std::tuple<double, double> obstacle, double obs_radius){
        // Intersection with obstacles calculate using the following method
        // PUT LINK TO MATH WRITTEN OUT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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
        return range;
    }

    // From https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
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

    // Subscribe to get the actual marker positions
    void real_obstacles_cb(const visualization_msgs::msg::MarkerArray & msg){
        obstacle_array = msg;
    }

///////////////////
    void heading_cb(const geometry_msgs::msg::Point & msg){
        heading = msg.z;
    }

    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr real_obstacles_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensed_obstacles_pub_;
    visualization_msgs::msg::MarkerArray obstacle_array;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double angle_min = 0.0, angle_max = 0.0, angle_increment = 0.0, min_range = 0.0, max_range = 0.0;

    sensor_msgs::msg::LaserScan laser_scan;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

    double obstacle_radius = 0.05; // Will have to subscribe to this later to make smarter AGHHHHHHHHH
    double arena_x_len = 5.0; // Will have to make param to actually get
    double arena_y_len = 5.0; // Will have to make param to actually get

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr heading_sub_;
    double heading = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLaser>());
    rclcpp::shutdown();
    return 0;
}
