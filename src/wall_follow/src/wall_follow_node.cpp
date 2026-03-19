#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <limits>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        declare_parameter("kp", 0.9);
        declare_parameter("ki", 0.0);
        declare_parameter("kd", 0.18);
        declare_parameter("desired_distance", 1.0);
        declare_parameter("lookahead_L", 1.0);
        declare_parameter("theta", 45.0);

        kp = get_parameter("kp").as_double();
        ki = get_parameter("ki").as_double();
        kd = get_parameter("kd").as_double();
        desired_distance_ = get_parameter("desired_distance").as_double();
        lookahead_L_ = get_parameter("lookahead_L").as_double();
        theta_ = get_parameter("theta").as_double() * M_PI / 180.0;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1)
        );

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        prev_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Wall Follow Node Initialized");
    }

private:
    //for tuning: Increase kd to damp oscillation and reduce overshoot
    // Only add a small amount of ki if needed
    double kp = 0.9;
    double kd = 0.18; //Have to tune during testing
    double ki = 0.0;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;
    double desired_distance_ = 1.0;
    double lookahead_L_ = 1.0;
    double theta_ = M_PI / 4.0;
    rclcpp::Time prev_time_;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;


    double get_range(const sensor_msgs::msg::LaserScan &scan, double angle) //changed parameter from range_data to &scan
    {

        if(angle < scan.angle_min || angle > scan.angle_max){ //Clamp the out of bound angles
            return std::numeric_limits<double>::infinity();
        }

        int index = (int)round((angle - scan.angle_min)/scan.angle_increment);

        if(index < 0 || index >= (int)scan.ranges.size()){ //check if index is valid
            return std::numeric_limits<double>::infinity();
        }

        float range = scan.ranges[index];
        if(std::isnan(range) || std::isinf(range)){ //check if range is valid
            return std::numeric_limits<double>::infinity();
        }

        if(range < scan.range_min || range > scan.range_max){
            return std::numeric_limits<double>::infinity();
        }

        return (double)range;
    }

    // side: -1.0 = right wall, +1.0 = left wall
    double get_error(const sensor_msgs::msg::LaserScan &scan, double dist, double side)
    {
        double a = get_range(scan, side * (M_PI / 2.0 - theta_));
        double b = get_range(scan, side * M_PI / 2.0);

        double angle = std::atan2(a * std::cos(theta_) - b, a * std::sin(theta_));
        double Dt = b * std::cos(angle);           // perpendicular distance to wall
        double Dtfuture = Dt + lookahead_L_ * std::sin(angle); // predicted future dist

        return dist - Dtfuture;
    }

    void pid_control(double error, double dt)
    {
        // Integral with anti-windup
        const double MAX_INTEGRAL = 1.0;
        integral = std::clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);

        double derivative = (dt > 0.0) ? (error - prev_error) / dt : 0.0;
        double steering_angle = kp * error + ki * integral + kd * derivative;
        prev_error = error;

        // Clamp to physical steering limit (~24 degrees)
        const double MAX_STEER = 0.4189;
        steering_angle = std::clamp(steering_angle, -MAX_STEER, MAX_STEER);

        // Velocity based on steering demand: large correction = slow down
        double abs_steer = std::abs(steering_angle);
        double velocity;
        if      (abs_steer < 0.1) velocity = 2.0;
        else if (abs_steer < 0.2) velocity = 1.5;
        else                      velocity = 0.8;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity;
        drive_pub_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        rclcpp::Time now = this->now();
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;

        // Dual-wall: centre between right and left walls
        double right_error = get_error(*scan_msg, desired_distance_, -1.0);
        double left_error  = get_error(*scan_msg, desired_distance_,  1.0);
        double error = right_error - left_error;

        pid_control(error, dt);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
