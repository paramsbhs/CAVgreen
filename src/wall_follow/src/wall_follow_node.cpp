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
        declare_parameter("speed_straight", 2.0);
        declare_parameter("speed_mild", 1.5);
        declare_parameter("speed_sharp", 0.8);
        declare_parameter("steer_thresh_mild", 0.1);
        declare_parameter("steer_thresh_sharp", 0.2);
        declare_parameter("derivative_filter_alpha", 0.3);

        kp = get_parameter("kp").as_double();
        ki = get_parameter("ki").as_double();
        kd = get_parameter("kd").as_double();
        desired_distance_ = get_parameter("desired_distance").as_double();
        lookahead_L_ = get_parameter("lookahead_L").as_double();
        theta_ = get_parameter("theta").as_double() * M_PI / 180.0;
        speed_straight_ = get_parameter("speed_straight").as_double();
        speed_mild_ = get_parameter("speed_mild").as_double();
        speed_sharp_ = get_parameter("speed_sharp").as_double();
        steer_thresh_mild_ = get_parameter("steer_thresh_mild").as_double();
        steer_thresh_sharp_ = get_parameter("steer_thresh_sharp").as_double();
        deriv_alpha_ = get_parameter("derivative_filter_alpha").as_double();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1)
        );

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        prev_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Wall Follow Node Initialized");
    }

private:
    // All tuning values come from hardware_params.yaml via declare_parameter() above.
    // Edit config/hardware_params.yaml — do not hardcode values here.
    double kp;
    double kd;
    double ki;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double filtered_derivative_ = 0.0;
    double integral = 0.0;
    double desired_distance_;
    double lookahead_L_;
    double theta_;
    double speed_straight_;
    double speed_mild_;
    double speed_sharp_;
    double steer_thresh_mild_;
    double steer_thresh_sharp_;
    double deriv_alpha_;
    double prev_steering_error_ = 0.0;  // tracks last error used for wall-mode blending
    rclcpp::Time prev_time_;
    bool first_scan_ = true;
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
    // Returns true if wall was detected, false if rays were invalid.
    // On detection, sets dtfuture_out to the predicted perpendicular distance.
    bool get_wall_dist(const sensor_msgs::msg::LaserScan &scan, double side, double &dtfuture_out)
    {
        double a = get_range(scan, side * (M_PI / 2.0 - theta_));
        double b = get_range(scan, side * M_PI / 2.0);

        if (!std::isfinite(a) || !std::isfinite(b)) {
            return false;
        }

        double angle = std::atan2(a * std::cos(theta_) - b, a * std::sin(theta_));
        // Clamp alpha so corner walls don't produce unrealistic heading estimates
        const double MAX_ALPHA = M_PI / 6.0; // 30 degrees
        angle = std::clamp(angle, -MAX_ALPHA, MAX_ALPHA);
        double Dt = b * std::cos(angle);           // perpendicular distance to wall
        dtfuture_out = Dt + lookahead_L_ * std::sin(angle);
        return true;
    }

    void pid_control(double error, double dt)
    {
        // Integral with anti-windup
        const double MAX_INTEGRAL = 1.0;
        integral = std::clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);

        // Low-pass filtered derivative to suppress LiDAR noise spikes.
        // alpha near 0 = heavy smoothing, near 1 = raw derivative.
        const double MIN_DT = 0.01;
        double raw_derivative = (dt >= MIN_DT) ? (error - prev_error) / dt : 0.0;
        filtered_derivative_ = deriv_alpha_ * raw_derivative
                             + (1.0 - deriv_alpha_) * filtered_derivative_;
        prev_error = error;
        double steering_angle = kp * error + ki * integral + kd * filtered_derivative_;

        // Clamp to physical steering limit (~24 degrees)
        const double MAX_STEER = 0.4189;
        steering_angle = std::clamp(steering_angle, -MAX_STEER, MAX_STEER);

        // Velocity based on steering demand: large correction = slow down
        double abs_steer = std::abs(steering_angle);
        double velocity;
        if      (abs_steer < steer_thresh_mild_)  velocity = speed_straight_;
        else if (abs_steer < steer_thresh_sharp_) velocity = speed_mild_;
        else                                      velocity = speed_sharp_;

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

        double dtf_right, dtf_left;
        bool right_ok = get_wall_dist(*scan_msg, -1.0, dtf_right);
        bool left_ok  = get_wall_dist(*scan_msg,  1.0, dtf_left);

        double error;
        if (right_ok && left_ok) {
            // Dual-wall: centre between both walls
            error = dtf_left - dtf_right;
        } else if (right_ok) {
            error = desired_distance_ - dtf_right;
        } else if (left_ok) {
            error = dtf_left - desired_distance_;
        } else {
            error = 0.0;  // no walls visible, go straight
        }

        // Rate-limit error changes to avoid steering jerks at wall transitions
        // (e.g. corridor opening into a room where one wall disappears).
        const double MAX_ERROR_JUMP = 0.15;  // max change per scan cycle
        double delta = error - prev_steering_error_;
        if (std::abs(delta) > MAX_ERROR_JUMP) {
            error = prev_steering_error_ + std::copysign(MAX_ERROR_JUMP, delta);
        }
        prev_steering_error_ = error;

        // Skip first scan so prev_error is seeded before derivative runs
        if (first_scan_) {
            prev_error = error;
            first_scan_ = false;
            return;
        }

        pid_control(error, dt);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
