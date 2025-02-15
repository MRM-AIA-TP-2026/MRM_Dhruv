#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance"), last_cmd_dir_(1) {
        // Parameter declaration and validation
        this->declare_parameters();
        this->validate_parameters();
        
        // Setup parameter callback
        param_handler_ = this->add_on_set_parameters_callback(
            std::bind(&ObstacleAvoidance::parameters_callback, this, std::placeholders::_1));

        // QoS setup for consistent laser scan reception
        auto qos = rclcpp::SensorDataQoS();
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&ObstacleAvoidance::scan_callback, this, std::placeholders::_1));
        
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Command timer for consistent control rate
        cmd_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObstacleAvoidance::publish_command, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node initialized");
    }

private:
    void declare_parameters() {
        this->declare_parameter("min_distance", 0.5);
        this->declare_parameter("linear_speed", 0.2);
        this->declare_parameter("max_angular_speed", 1.5);
        this->declare_parameter("scan_angle_range", 90.0);  // Degrees
        this->declare_parameter("obstacle_threshold", 0.3);
        this->declare_parameter("safety_ratio", 0.3);
        this->declare_parameter("control_rate", 10.0);
    }

    void validate_parameters() {
        min_distance_ = this->get_parameter("min_distance").as_double();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        scan_angle_range_ = this->get_parameter("scan_angle_range").as_double();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
        safety_ratio_ = this->get_parameter("safety_ratio").as_double();
        
        // Parameter validation
        if (min_distance_ <= 0) {
            RCLCPP_WARN(this->get_logger(), 
                "Invalid min_distance %.2f, using default 0.5", min_distance_);
            min_distance_ = 0.5;
        }
        if (linear_speed_ < 0) {
            RCLCPP_WARN(this->get_logger(), 
                "Negative linear speed %.2f, using absolute value", linear_speed_);
            linear_speed_ = std::abs(linear_speed_);
        }
        scan_angle_range_ = std::clamp(scan_angle_range_, 10.0, 180.0);
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto &param : parameters) {
            if (param.get_name() == "min_distance" && param.as_double() <= 0) {
                result.successful = false;
                result.reason = "min_distance must be positive";
            }
            // Add more parameter checks as needed
        }
        return result;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        current_scan_ = *msg;
        scan_updated_ = true;
    }

    void publish_command() {
        if (!scan_updated_) return;

        std::lock_guard<std::mutex> lock(scan_mutex_);
        const auto& msg = current_scan_;
        
        // Calculate detection sector
        const double scan_half_range = (scan_angle_range_ / 2) * (M_PI / 180);
        const auto [left_min, right_min] = calculate_safety_distances(msg, scan_half_range);
        
        geometry_msgs::msg::Twist cmd_vel;
        
        if (left_min < min_distance_ || right_min < min_distance_) {
            // Obstacle detected - decide turning direction
            const double turn_direction = (right_min > left_min) ? -1 : 1;
            last_cmd_dir_ = turn_direction;
            
            // Proportional control based on closest obstacle
            const double min_dist = std::min(left_min, right_min);
            const double angular_speed = std::min(
                max_angular_speed_, 
                (min_distance_ - min_dist) / min_distance_ * max_angular_speed_);
            
            cmd_vel.angular.z = angular_speed * turn_direction;
            cmd_vel.linear.x = linear_speed_ * safety_ratio_;
            
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Avoiding obstacle: turning %.2f rad/s", cmd_vel.angular.z);
        } else {
            // Clear path - move forward
            cmd_vel.linear.x = linear_speed_;
            cmd_vel.angular.z = 0;
        }
        
        velocity_publisher_->publish(cmd_vel);
        scan_updated_ = false;
    }

    std::pair<double, double> calculate_safety_distances(
        const sensor_msgs::msg::LaserScan& scan, double scan_half_range) {
        const int num_samples = scan.ranges.size();
        if (num_samples == 0) return {INFINITY, INFINITY};

        const int mid_index = num_samples / 2;
        const int sector_size = static_cast<int>((scan_half_range / scan.angle_increment));
        
        // Calculate left and right sectors
        double left_min = INFINITY;
        double right_min = INFINITY;
        
        for (int i = 0; i < sector_size; ++i) {
            // Right sector (negative angles)
            int right_idx = mid_index - i;
            if (right_idx >= 0 && right_idx < num_samples) {
                if (is_valid_range(scan.ranges[right_idx])) {
                    right_min = std::min(right_min, scan.ranges[right_idx]);
                }
            }
            
            // Left sector (positive angles)
            int left_idx = mid_index + i;
            if (left_idx >= 0 && left_idx < num_samples) {
                if (is_valid_range(scan.ranges[left_idx])) {
                    left_min = std::min(left_min, scan.ranges[left_idx]);
                }
            }
        }
        
        return {left_min, right_min};
    }

    bool is_valid_range(float range) const {
        return std::isfinite(range) && 
               range >= current_scan_.range_min && 
               range <= current_scan_.range_max;
    }

    // ROS 2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_handler_;
    
    // State variables
    sensor_msgs::msg::LaserScan current_scan_;
    std::mutex scan_mutex_;
    bool scan_updated_ = false;
    int last_cmd_dir_;

    // Parameters
    double min_distance_;
    double linear_speed_;
    double max_angular_speed_;
    double scan_angle_range_;
    double obstacle_threshold_;
    double safety_ratio_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}