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
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1)
        );

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    
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
    double error = 0.0;
    double integral = 0.0;

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

        if(index < 0 || index >= scan.ranges.size()){ //check if index is valid
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

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        return 0.0;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID
        double b = get_range(*scan_msg, M_PI/2.0);

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
