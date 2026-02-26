#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode() : Node("safety_node")
    {
        // Declare parameters
        this->declare_parameter("ttc_threshold", 0.5);
        ttc_threshold_ = this->get_parameter("ttc_threshold").as_double();

        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));

        // Publisher
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        RCLCPP_INFO(this->get_logger(), "Safety Node Initialized. TTC Threshold: %.2f seconds", ttc_threshold_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get longitudinal velocity
        current_speed_ = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Only check if moving forward
        if (current_speed_ < 0.1) {
            return;
        }

        // Calculate iTTC for each beam
        std::vector<double> iTTC_array;
        iTTC_array.reserve(msg->ranges.size());

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double range = msg->ranges[i];

            // Skip invalid measurements
            if (std::isnan(range) || std::isinf(range) ||
                range < msg->range_min || range > msg->range_max) {
                iTTC_array.push_back(std::numeric_limits<double>::infinity());
                continue;
            }

            // Calculate beam angle
            double angle = msg->angle_min + i * msg->angle_increment;

            // Calculate range rate: r_dot = -v * cos(theta)
            // Negative because range is decreasing as we approach obstacle
            double range_rate = current_speed_ * std::cos(angle);

            // Calculate iTTC = r / {-r_dot}+
            // Only meaningful when approaching (range_rate > 0)
            double iTTC;
            if (range_rate > 0) {
                iTTC = range / range_rate;
            } else {
                iTTC = std::numeric_limits<double>::infinity();
            }

            iTTC_array.push_back(iTTC);
        }

        // Find minimum iTTC
        double min_iTTC = *std::min_element(iTTC_array.begin(), iTTC_array.end());

        // Check if collision is imminent
        if (min_iTTC < ttc_threshold_) {
            // Emergency brake
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->now();
            drive_msg.drive.speed = 0.0;

            drive_pub_->publish(drive_msg);

            RCLCPP_WARN(this->get_logger(), "EMERGENCY BRAKE! Min iTTC: %.3f seconds", min_iTTC);
        }
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // Variables
    double current_speed_ = 0.0;
    double ttc_threshold_ = 0.5;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}
