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
        // max_range is deliberately short: depth saturates so the plateau
        // logic aims at the center of wide free regions instead of chasing
        // deep slivers at the FOV edges (which causes steering flip-flop at
        // junctions). The FOV is < 180 deg for the same reason: beams at
        // +/-90 deg look straight down the corridor walls and tie with real
        // openings.
        declare_parameter("max_range", 4.0);
        declare_parameter("smoothing_window", 5);
        declare_parameter("bubble_radius_m", 0.25);
        declare_parameter("disparity_threshold_m", 0.3);
        declare_parameter("forward_fov_deg", 170.0);
        declare_parameter("steer_alpha", 0.35);  // steering low-pass factor
        // The safety bubble is an emergency mask for imminent collision, not
        // a routine filter: applying it to ordinary side walls alternately
        // erases the left/right half of the scan and makes the gap target
        // flip-flop at junctions.
        declare_parameter("bubble_trigger_dist", 0.6);
        declare_parameter("reverse_speed", 0.7);
        declare_parameter("reverse_enter_clearance", 0.3);
        declare_parameter("reverse_exit_clearance", 0.8);
        declare_parameter("max_steering_angle", 0.4189);
        declare_parameter("speed_straight", 3.5);
        declare_parameter("speed_mild", 2.0);
        declare_parameter("speed_sharp", 1.0);
        declare_parameter("steer_thresh_mild", 0.1745);   // 10 deg
        declare_parameter("steer_thresh_sharp", 0.3491);  // 20 deg
        declare_parameter("clearance_speed_gain", 1.5);   // m/s per m of headway
        declare_parameter("clearance_safe_dist", 0.3);
        declare_parameter("clearance_cone_deg", 10.0);
        declare_parameter("speed_min", 0.5);

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
        clearance_gain_    = get_parameter("clearance_speed_gain").as_double();
        clearance_safe_    = get_parameter("clearance_safe_dist").as_double();
        clearance_cone_    = get_parameter("clearance_cone_deg").as_double() * M_PI / 180.0;
        speed_min_         = get_parameter("speed_min").as_double();
        steer_alpha_       = get_parameter("steer_alpha").as_double();
        bubble_trigger_    = get_parameter("bubble_trigger_dist").as_double();
        reverse_speed_     = get_parameter("reverse_speed").as_double();
        reverse_enter_     = get_parameter("reverse_enter_clearance").as_double();
        reverse_exit_      = get_parameter("reverse_exit_clearance").as_double();

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
    double clearance_gain_, clearance_safe_, clearance_cone_, speed_min_;
    double steer_alpha_;
    double steer_state_ = 0.0;
    double bubble_trigger_;
    double reverse_speed_, reverse_enter_, reverse_exit_;
    bool   reversing_ = false;

    int start_idx_ = -1;
    int end_idx_   = -1;
    int center_idx_ = 0;  // window-local index of the 0-rad (straight ahead) beam
    size_t cached_scan_size_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    void compute_forward_window(const sensor_msgs::msg::LaserScan &scan)
    {
        if (start_idx_ >= 0 && scan.ranges.size() == cached_scan_size_) return;
        cached_scan_size_ = scan.ranges.size();
        double half = forward_fov_ / 2.0;
        double a_lo = std::max((double)scan.angle_min, -half);
        double a_hi = std::min((double)scan.angle_max,  half);
        start_idx_ = (int)std::round((a_lo - scan.angle_min) / scan.angle_increment);
        end_idx_   = (int)std::round((a_hi - scan.angle_min) / scan.angle_increment);
        start_idx_ = std::clamp(start_idx_, 0, (int)scan.ranges.size() - 1);
        end_idx_   = std::clamp(end_idx_,   0, (int)scan.ranges.size() - 1);
        center_idx_ = (int)std::round((0.0 - scan.angle_min) / scan.angle_increment) - start_idx_;
        center_idx_ = std::clamp(center_idx_, 0, end_idx_ - start_idx_);
    }

    // Sliding-window mean with max-range clip. NaN and sub-range_min dropouts
    // (real lidars report 0.0 for missed returns) are excluded from the mean;
    // +inf means "no obstacle within lidar range" and counts as max_range so
    // open space is not mistaken for an obstacle.
    std::vector<float> preprocess_lidar(const std::vector<float> &ranges, float range_min)
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
                if (std::isnan(r)) continue;
                if (r > (float)max_range_) r = (float)max_range_;
                if (r < range_min) continue;
                sum += r;
                ++cnt;
            }
            proc[i] = (cnt > 0) ? (float)(sum / cnt) : 0.0f;
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

    // "Better idea": aim for the deepest region of the gap. Beams tied at max
    // depth can form several disjoint plateaus (e.g. deep on both sides of a
    // far obstacle), so take the middle of the *widest contiguous* plateau —
    // the midpoint of first/last deep beam could land on the obstacle between.
    int find_best_point(int gap_start, int gap_end, const std::vector<float> &proc)
    {
        float best = -1.0f;
        for (int i = gap_start; i <= gap_end; ++i) best = std::max(best, proc[i]);
        const float thresh = best - 1e-3f;
        int best_s = gap_start, best_e = gap_start, best_len = 0;
        int cur_s = -1;
        for (int i = gap_start; i <= gap_end + 1; ++i) {
            if (i <= gap_end && proc[i] >= thresh) {
                if (cur_s < 0) cur_s = i;
            } else if (cur_s >= 0) {
                int len = i - cur_s;
                if (len > best_len) { best_len = len; best_s = cur_s; best_e = i - 1; }
                cur_s = -1;
            }
        }
        return (best_s + best_e) / 2;
    }

    // Min range within a narrow cone about the car's heading. Must be called
    // BEFORE zero_safety_bubble: bubbled beams read 0 and would be skipped
    // here, making a dead-ahead obstacle look like open space. Zeros from
    // preprocessing (no valid returns) are still skipped.
    double forward_clearance(const std::vector<float> &proc, double angle_increment)
    {
        const int n = (int)proc.size();
        int center = center_idx_;
        int span = (int)std::ceil(clearance_cone_ / angle_increment);
        double lo = max_range_;
        for (int i = std::max(0, center - span); i <= std::min(n - 1, center + span); ++i) {
            if (proc[i] > 0.0f) lo = std::min(lo, (double)proc[i]);
        }
        return lo;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        compute_forward_window(*scan_msg);

        auto proc = preprocess_lidar(scan_msg->ranges, scan_msg->range_min);
        apply_disparity_extender(proc, scan_msg->angle_increment);

        int closest = 0;
        float min_r = std::numeric_limits<float>::infinity();
        for (int i = 0; i < (int)proc.size(); ++i) {
            if (proc[i] > 0.0f && proc[i] < min_r) { min_r = proc[i]; closest = i; }
        }
        double ahead = forward_clearance(proc, scan_msg->angle_increment);

        if (min_r < bubble_trigger_)
            zero_safety_bubble(proc, closest, scan_msg->angle_increment);

        int gap_s, gap_e;
        find_max_gap(proc, gap_s, gap_e);

        // Recovery: nose blocked (or scan fully masked) — back out straight
        // until there is room to steer again. Hysteresis prevents chatter.
        if (reversing_ || gap_e <= gap_s || ahead < reverse_enter_) {
            reversing_ = (gap_e <= gap_s) || (ahead < reverse_exit_);
            if (reversing_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Blocked (clearance %.2f m), reversing", ahead);
                ackermann_msgs::msg::AckermannDriveStamped rev;
                rev.header.stamp = now();
                rev.drive.speed = -reverse_speed_;
                rev.drive.steering_angle = 0.0;
                drive_pub_->publish(rev);
                steer_state_ = 0.0;
                return;
            }
        }
        int best = find_best_point(gap_s, gap_e, proc);

        int global_idx = start_idx_ + best;
        double steering = scan_msg->angle_min + global_idx * scan_msg->angle_increment;
        steering = std::clamp(steering, -max_steer_, max_steer_);
        steer_state_ += steer_alpha_ * (steering - steer_state_);
        steering = steer_state_;

        double abs_steer = std::abs(steering);
        double speed;
        if      (abs_steer < steer_thresh_mild_)  speed = speed_straight_;
        else if (abs_steer < steer_thresh_sharp_) speed = speed_mild_;
        else                                      speed = speed_sharp_;

        // Brake for whatever is straight ahead regardless of where we are
        // aiming: steering-based speed alone reacts only once the turn has
        // already started, which is too late entering a tight corner.
        double v_clear = clearance_gain_ * (ahead - clearance_safe_);
        speed = std::clamp(std::min(speed, v_clear), speed_min_, speed_straight_);

        ackermann_msgs::msg::AckermannDriveStamped drive;
        drive.header.stamp = now();
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
