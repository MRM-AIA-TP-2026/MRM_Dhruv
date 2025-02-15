#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance") {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleAvoidance::scan_callback, this, std::placeholders::_1));
        
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        bool obstacle_detected = false;
        double min_distance = 0.5;  // Minimum safe distance

        for (const auto& range : msg->ranges) {
            if (range < min_distance) {
                obstacle_detected = true;
                break;
            }
        }

        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        if (obstacle_detected) {
            cmd_vel_msg.angular.z = 0.5;  // Rotate
            cmd_vel_msg.linear.x = 0.0;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Rotating...");
        } else {
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_msg.linear.x = 0.2;  // Move forward
        }
        velocity_publisher_->publish(cmd_vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
