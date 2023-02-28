#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <random>
#include "tf2_ros/transform_listener.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "tf2_ros/buffer.h"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlelib/rigid2d.hpp"

using namespace std::chrono_literals;

class BasicSensor : public rclcpp::Node
{
public:
    BasicSensor()
    : Node("basic_sensor"), count_(0)
    {
        // Define parameter to change basic sensor noise
        auto basic_sensor_variance_desc = rcl_interfaces::msg::ParameterDescriptor();
        basic_sensor_variance_desc.description = "Variance to change sensor noise for basic sensor";
        declare_parameter("basic_sensor_variance", 0.01, basic_sensor_variance_desc);
        basic_sensor_variance = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();

        // Define parameter to change maximum range of the sensor
        auto max_range_desc = rcl_interfaces::msg::ParameterDescriptor();
        max_range_desc.description = "Maximum range for basic sensor (m)";
        declare_parameter("max_range", 1.5, max_range_desc);
        max_range = get_parameter("max_range").get_parameter_value().get<double>();

        // Create 5 Hz timer
        auto hz_in_ms = std::chrono::milliseconds((long)(1000 / (5.0)));
        timer_ = create_wall_timer(
        hz_in_ms, std::bind(&BasicSensor::timer_callback, this));

        // Create listener to get red robot location
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create subcriber to get actual obstacle positions
         real_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/nusim/obstacles", 10, std::bind(&BasicSensor::real_obstacles_cb, this, std::placeholders::_1));

        // Publisher to publish sensed obstacles
        sensed_obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker_array", 10);

        // temp point pub to get heading
         heading_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "heading", 10, std::bind(&BasicSensor::heading_cb, this, std::placeholders::_1));
    }

private:

    // Timer callback going at 5 hz
    void timer_callback()
    {
        // Check position of the robot 
        // Look up for the transformation between nusim/world and the red robot frames
        geometry_msgs::msg::TransformStamped t;
        try {
          t = tf_buffer_->lookupTransform(
            "nusim/world", "red/base_footprint",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform red/base_footprint to nusim/world");
          return;
        }
        turtlelib::Transform2D Twr{{t.transform.translation.x, t.transform.translation.y}, heading};
        turtlelib::Transform2D Trw = Twr.inv();

        // Get a Gaussian distributin of noise
        std::normal_distribution<> gaus_dist(0.0, basic_sensor_variance);

        // Loop through each of the sensed obstacles in the marker array
        for(int i=0; i < (int)sensed_obstacles.markers.size(); i++){
            turtlelib::Transform2D Two{{sensed_obstacles.markers.at(i).pose.position.x, sensed_obstacles.markers.at(i).pose.position.y}};
            turtlelib::Transform2D Tro = Trw*Two;

            // Check if the obstacle intersects with the max range of the robot

            // Add noise to the position of the obstacle'
            sensed_obstacles.markers.at(i).header.frame_id = "red/base_footprint";
            sensed_obstacles.markers.at(i).header.stamp = get_clock()->now();
            sensed_obstacles.markers.at(i).pose.position.x =Tro.translation().x+ gaus_dist(get_random());
            sensed_obstacles.markers.at(i).pose.position.y =Tro.translation().y + gaus_dist(get_random());

            // Caclulate the straight line distance between center of robot to center of obstacle
            double dist_btw_centers = std::sqrt(
                (Tro.translation().x*Tro.translation().x)
                + (Tro.translation().y*Tro.translation().y));

            // If the straight line distance minus the obsacle radius is less than the max radius of
            // the robot than the obstacle is in range -> ADD
            if(dist_btw_centers < max_range){
                sensed_obstacles.markers.at(i).action = visualization_msgs::msg::Marker::ADD;

            }
            //Else -> DELETE
            else{sensed_obstacles.markers.at(i).action = visualization_msgs::msg::Marker::DELETE;}

            // Change color to yellow
            sensed_obstacles.markers.at(i).color.r = 0.8;
            sensed_obstacles.markers.at(i).color.g = 0.8;
            sensed_obstacles.markers.at(i).color.b = 0.3;
        }

        // Publish marker array
        sensed_obstacles_pub_->publish(sensed_obstacles);
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
        sensed_obstacles = msg;
    }

    void heading_cb(const geometry_msgs::msg::Point & msg){
        heading = msg.z;
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr heading_sub_;
    double heading = 0.0;
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr real_obstacles_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensed_obstacles_pub_;
    visualization_msgs::msg::MarkerArray sensed_obstacles;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double basic_sensor_variance = 0.0, max_range = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicSensor>());
    rclcpp::shutdown();
    return 0;
}
