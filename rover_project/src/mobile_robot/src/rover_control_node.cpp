#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>  // Include the chrono header for std::chrono_literals

using namespace std::chrono_literals;  // Enable the usage of time literals like 100ms

class RoverControlNode : public rclcpp::Node
{
public:
    RoverControlNode() : Node("rover_control_node")
    {
        // Subscriber to the target bearing
        target_bearing_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "target_bearing", 10, std::bind(&RoverControlNode::bearing_callback, this, std::placeholders::_1));

        // Publisher to control the rover
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Timer to update rover's control periodically
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RoverControlNode::timer_callback, this));  // Now using the correct time literal
    }

private:
    void bearing_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Update the target bearing
        target_bearing_ = msg->data;
    }

    void timer_callback()
    {
        // Compute the error in bearing and adjust the rover's motion
        double error = target_bearing_ - current_bearing_;

        // Normalize the error to the range [-180, 180] for shortest turn direction
        if (error > 180.0) error -= 360.0;
        if (error < -180.0) error += 360.0;

        // Control the rover's turning speed based on the error
        geometry_msgs::msg::Twist cmd_vel_msg;

        // Proportional control: Adjust the angular velocity
        cmd_vel_msg.angular.z = 0.5 * error;  // Adjust the factor as needed for speed
        cmd_vel_msg.linear.x = 0.2;           // Constant linear velocity

        // Publish the velocity command to cmd_vel
        velocity_publisher_->publish(cmd_vel_msg);

        // Update the current bearing after command application
        current_bearing_ += cmd_vel_msg.angular.z * 0.1;  // Adjust based on timestep (simulation)

        // Keep the current bearing within the range [0, 360]
        if (current_bearing_ >= 360.0) current_bearing_ -= 360.0;
        if (current_bearing_ < 0.0) current_bearing_ += 360.0;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_bearing_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_bearing_ = 0.0;   // Initial bearing of the rover
    double target_bearing_ = 0.0;    // Target bearing from input
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverControlNode>());
    rclcpp::shutdown();
    return 0;
}
