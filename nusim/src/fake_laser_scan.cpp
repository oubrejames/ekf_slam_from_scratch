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

using namespace std::chrono_literals;

class FakeLaser : public rclcpp::Node
{
public:
    FakeLaser()
    : Node("fake_laser"), count_(0)
    {
        // // Define parameter to change basic sensor noise
        // auto basic_sensor_variance_desc = rcl_interfaces::msg::ParameterDescriptor();
        // basic_sensor_variance_desc.description = "Variance to change sensor noise for basic sensor";
        // declare_parameter("basic_sensor_variance", 0.01, basic_sensor_variance_desc);
        // basic_sensor_variance = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();

        // // Define parameter to change maximum range of the sensor
        // auto max_range_desc = rcl_interfaces::msg::ParameterDescriptor();
        // max_range_desc.description = "Maximum range for basic sensor (m)";
        // declare_parameter("max_range", 1.0, max_range_desc);
        // max_range = get_parameter("max_range").get_parameter_value().get<double>();

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
        sensed_obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker_array", 10);
    }

private:

    // Timer callback going at 5 hz
    void timer_callback()
    {
        // Check position of the robot 
        // Look up for the transformation between nusim/world and the laser scanner frames
        geometry_msgs::msg::TransformStamped nuw2ls;
        try {
          nuw2ls = tf_buffer_->lookupTransform(
            "nusim/world", "red/base_scan",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform red/base_scan to nusim/world");
          return;
        }
        
        // Get a Gaussian distributin of noise
        std::normal_distribution<> gaus_dist(0.0, basic_sensor_variance);

    }

    std::tuple<double, double, bool> check_laser_intersect(double x1, double y1, double x2, double y2){
        // Intersection with obstacles calculate using the following method
        // https://mathworld.wolfram.com/Circle-LineIntersection.html

        // Calculate distance from lidar to max range point and determinent
        auto dx = x2 - x1;
        auto dy = y2 - y1;
        auto dr = std::sqrt(dx*dx+dy*dy);
        auto det = x1*y2 - x2*y1;
        double sign = 0.0;

        // Apply appropriate sign
        if (dy < 0){
            sign = -1.0;
        }
        else {sign = 1.0;}

        // Calculate possible x and y componenets for both plus and minus
        double xp = (det*dy+sign*dx*std::sqrt(obs_radius*obs_radius*dr*dr - det*det))/(dr*dr);
        double xm = (det*dy-sign*dx*std::sqrt(obs_radius*obs_radius*dr*dr - det*det))/(dr*dr);

        double yp = (-det*dx+abs(dy)*std::sqrt(obs_radius*obs_radius*dr*dr - det*det))/(dr*dr);
        double ym = (-det*dx-abs(dy)*std::sqrt(obs_radius*obs_radius*dr*dr - det*det))/(dr*dr);

        // Define points to return later
        double x = 0.0, y = 0.0;
        // Check which point is closer to the robot
        if(abs(xp-x1) < abs(xm-x1)){
            // If xp is closer, set intersect point to xp
            x = xp;
        }
        // Else, xm is closer and set intersect point to xm
        else{ x = xm;}

        if(abs(xp-x1) < abs(xm-x1)){
            // If yp is closer, set intersect point to yp
            y = yp;
        }
        // Else, ym is closer and set intersect point to ym
        else{ y = ym;}


        // Calculate discriminant
        double disc = obs_radius*obs_radius*dr*dr - det*det;

        // If discriminant is < 0 than you are not intersecting with an obstacle but could still be hitting wall 
        if ((disc < 0.0) | (dr < min_range)){

            // Check if intersecting with wall (checking if within bounds)
            if ((abs(x2) > arena_x_len/2.0 - 0.1) | (abs(y2) > arena_y_len/2.0 - 0.1 ))
            {
                // if the x is out of bounds update the x to return to be point on wall, else keep same
                if (abs(x2) > arena_x_len/2.0 - 0.1){
                    x = sgn(x2)*(arena_x_len/2.0 - 0.1);
                } else {x = x2;}

                // if the y is out of bounds update the x to return to be point on wall
                if (abs(y2) > arena_y_len/2.0 - 0.1){
                    y = sgn(y2)*(arena_y_len/2.0 - 0.1);
                } else {y = y2;}

                return {x, y, true};
            }
            // Not intersection at all - return false with huge value
            else{return {999, 999, false};}
        }

        // If discriminant is => 0 then there is interestion
        else if( disc >= 0.0){
            return {x, y, true};
        }
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
        sensed_obstacles = msg;
    }

    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr real_obstacles_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensed_obstacles_pub_;
    visualization_msgs::msg::MarkerArray sensed_obstacles;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double basic_sensor_variance = 0.0, max_range = 0.0;

    double obs_radius = 0.05; // Will have to subscribe to this later to make smarter AGHHHHHHHHH
    double min_range = 0.01; // Will have to make this a param
    double arena_x_len = 5.0; // Will have to make param to actually get
    double arena_y_len = 5.0/2.0 - 0.1; // Will have to make param to actually get

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLaser>());
    rclcpp::shutdown();
    return 0;
}
