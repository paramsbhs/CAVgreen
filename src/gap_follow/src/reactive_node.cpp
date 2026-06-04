#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        declare_parameter("max_range", 3.0);
        declare_parameter("smoothing_window", 5);
        declare_parameter("bubble_radius_m", 0.18);
        declare_parameter("disparity_threshold_m", 0.3);
        declare_parameter("forward_fov_deg", 180.0);
        declare_parameter("max_steering_angle", 0.4189);
        declare_parameter("speed_straight", 3.5);
        declare_parameter("speed_mild", 2.0);
        declare_parameter("speed_sharp", 1.0);
        declare_parameter("steer_thresh_mild", 0.1745);   // 10 deg
        declare_parameter("steer_thresh_sharp", 0.3491);  // 20 deg

        max_range_         = get_parameter("max_range").as_double();
        smoothing_window_  = get_parameter("smoothing_window").as_int();
        bubble_radius_m_   = get_parameter("bubble_radius_m").as_double();
        disparity_thresh_  = get_parameter("disparity_threshold_m").as_double();
        forward_fov_       = get_parameter("forward_fov_deg").as_double() * M_PI / 180.0;
        max_steer_         = get_parameter("max_steering_angle").as_double();
        speed_straight_    = get_parameter("speed_straight").as_double();
        speed_mild_        = get_parameter("speed_mild").as_double();
        speed_sharp_       = get_parameter("speed_sharp").as_double();
        steer_thresh_mild_ = get_parameter("steer_thresh_mild").as_double();
        steer_thresh_sharp_= get_parameter("steer_thresh_sharp").as_double();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10,
            std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        RCLCPP_INFO(get_logger(), "Reactive gap-follow node initialized");
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    double max_range_;
    int    smoothing_window_;
    double bubble_radius_m_;
    double disparity_thresh_;
    double forward_fov_;
    double max_steer_;
    double speed_straight_, speed_mild_, speed_sharp_;
    double steer_thresh_mild_, steer_thresh_sharp_;

    int start_idx_ = -1;
    int end_idx_   = -1;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    void compute_forward_window(const sensor_msgs::msg::LaserScan &scan)
    {
        if (start_idx_ >= 0) return;
        double half = forward_fov_ / 2.0;
        double a_lo = std::max((double)scan.angle_min, -half);
        double a_hi = std::min((double)scan.angle_max,  half);
        start_idx_ = (int)std::round((a_lo - scan.angle_min) / scan.angle_increment);
        end_idx_   = (int)std::round((a_hi - scan.angle_min) / scan.angle_increment);
        start_idx_ = std::clamp(start_idx_, 0, (int)scan.ranges.size() - 1);
        end_idx_   = std::clamp(end_idx_,   0, (int)scan.ranges.size() - 1);
    }

    // Sliding-window mean with NaN/inf rejection and max-range clip.
    std::vector<float> preprocess_lidar(const std::vector<float> &ranges)
    {
        const int n = end_idx_ - start_idx_ + 1;
        std::vector<float> proc(n, 0.0f);
        const int half = smoothing_window_ / 2;
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            int    cnt = 0;
            for (int k = -half; k <= half; ++k) {
                int src = start_idx_ + i + k;
                if (src < start_idx_ || src > end_idx_) continue;
                float r = ranges[src];
                if (std::isnan(r) || std::isinf(r)) continue;
                sum += r;
                ++cnt;
            }
            float v = (cnt > 0) ? (float)(sum / cnt) : 0.0f;
            proc[i] = std::min(v, (float)max_range_);
        }
        return proc;
    }

    // Disparity extender: at every sharp depth jump, project the near reading
    // sideways by one car half-width so the gap-finder can't aim through a corner.
    void apply_disparity_extender(std::vector<float> &proc, double angle_increment)
    {
        const int n = (int)proc.size();
        if (n < 2) return;
        std::vector<float> out = proc;
        for (int i = 0; i < n - 1; ++i) {
            float a = proc[i], b = proc[i + 1];
            if (std::abs(a - b) <= disparity_thresh_) continue;
            float near_r = std::min(a, b);
            int span = (int)std::ceil(std::asin(std::min(1.0, bubble_radius_m_ / std::max(0.1f, near_r))) / angle_increment);
            if (a < b) {
                for (int k = 0; k <= span && i + k < n; ++k) out[i + k] = std::min(out[i + k], near_r);
            } else {
                for (int k = 0; k <= span && i + 1 - k >= 0; ++k) out[i + 1 - k] = std::min(out[i + 1 - k], near_r);
            }
        }
        proc = std::move(out);
    }

    void zero_safety_bubble(std::vector<float> &proc, int closest, double angle_increment)
    {
        float r = proc[closest];
        if (r <= 0.0f) return;
        int span = (int)std::ceil(std::asin(std::min(1.0, bubble_radius_m_ / std::max(0.1f, r))) / angle_increment);
        int lo = std::max(0, closest - span);
        int hi = std::min((int)proc.size() - 1, closest + span);
        for (int i = lo; i <= hi; ++i) proc[i] = 0.0f;
    }

    void find_max_gap(const std::vector<float> &proc, int &gap_start, int &gap_end)
    {
        const int n = (int)proc.size();
        int best_s = 0, best_e = 0, best_len = 0;
        int cur_s = -1;
        for (int i = 0; i < n; ++i) {
            if (proc[i] > 0.0f) {
                if (cur_s < 0) cur_s = i;
                int cur_len = i - cur_s + 1;
                if (cur_len > best_len) { best_len = cur_len; best_s = cur_s; best_e = i; }
            } else {
                cur_s = -1;
            }
        }
        gap_start = best_s;
        gap_end   = best_e;
    }

    // "Better idea": aim for the deepest point in the gap; if multiple beams tie at
    // max range, pick the middle index of that plateau (more stable than argmax).
    int find_best_point(int gap_start, int gap_end, const std::vector<float> &proc)
    {
        float best = -1.0f;
        for (int i = gap_start; i <= gap_end; ++i) best = std::max(best, proc[i]);
        int plateau_s = gap_start, plateau_e = gap_end;
        for (int i = gap_start; i <= gap_end; ++i) {
            if (proc[i] >= best - 1e-3f) { plateau_s = i; break; }
        }
        for (int i = gap_end; i >= gap_start; --i) {
            if (proc[i] >= best - 1e-3f) { plateau_e = i; break; }
        }
        return (plateau_s + plateau_e) / 2;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        compute_forward_window(*scan_msg);

        auto proc = preprocess_lidar(scan_msg->ranges);
        apply_disparity_extender(proc, scan_msg->angle_increment);

        int closest = 0;
        float min_r = std::numeric_limits<float>::infinity();
        for (int i = 0; i < (int)proc.size(); ++i) {
            if (proc[i] > 0.0f && proc[i] < min_r) { min_r = proc[i]; closest = i; }
        }
        zero_safety_bubble(proc, closest, scan_msg->angle_increment);

        int gap_s, gap_e;
        find_max_gap(proc, gap_s, gap_e);
        if (gap_e <= gap_s) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "No gap found, braking");
            ackermann_msgs::msg::AckermannDriveStamped stop;
            stop.drive.speed = 0.0;
            drive_pub_->publish(stop);
            return;
        }
        int best = find_best_point(gap_s, gap_e, proc);

        int global_idx = start_idx_ + best;
        double steering = scan_msg->angle_min + global_idx * scan_msg->angle_increment;
        steering = std::clamp(steering, -max_steer_, max_steer_);

        double abs_steer = std::abs(steering);
        double speed;
        if      (abs_steer < steer_thresh_mild_)  speed = speed_straight_;
        else if (abs_steer < steer_thresh_sharp_) speed = speed_mild_;
        else                                      speed = speed_sharp_;

        ackermann_msgs::msg::AckermannDriveStamped drive;
        drive.drive.steering_angle = steering;
        drive.drive.speed = speed;
        drive_pub_->publish(drive);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
