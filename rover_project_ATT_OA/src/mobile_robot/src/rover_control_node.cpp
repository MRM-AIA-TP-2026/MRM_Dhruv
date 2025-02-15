#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"  
#include "sensor_msgs/msg/nav_sat_fix.hpp"  // Added for GPS data
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  
#include <cmath>
#include <chrono>
#include <limits>  

using namespace std::chrono_literals;

class RoverControlNode : public rclcpp::Node {
public:
    RoverControlNode() : Node("rover_control_node"), gps_initialized_(false) {  // ✅ Initialize gps_initialized_
        target_bearing_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "target_bearing", 10, std::bind(&RoverControlNode::bearing_callback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&RoverControlNode::imu_callback, this, std::placeholders::_1)); 

        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&RoverControlNode::gps_callback, this, std::placeholders::_1)); // ✅ GPS Subscription

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            500ms, std::bind(&RoverControlNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Rover Control Node initialized");
    }

private:
    void bearing_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        target_bearing_ = msg->data;

        // Store the last received target coordinates
        target_latitude_ = current_latitude_;
        target_longitude_ = current_longitude_;

        RCLCPP_INFO(this->get_logger(), "Received Target Bearing: %.2f degrees", target_bearing_);
    }


    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!gps_initialized_) {
            reference_latitude_ = msg->latitude;
            reference_longitude_ = msg->longitude;
            gps_initialized_ = true;
        }

        // Ignore minor changes in GPS (noise filtering)
        if (std::abs(msg->latitude - current_latitude_) < 0.000005 &&
            std::abs(msg->longitude - current_longitude_) < 0.000005) {
            return;
        }

        current_latitude_ = msg->latitude;
        current_longitude_ = msg->longitude;
        RCLCPP_INFO(this->get_logger(), "Received GPS: Lat: %.6f, Lon: %.6f", current_latitude_, current_longitude_);
    }




    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double imu_yaw = yaw * (180.0 / M_PI);
        if (imu_yaw < 0) imu_yaw += 360.0;

        // Only update if the change is significant (>1 degree)
        if (std::abs(imu_yaw - current_bearing_) > 1.0) {
            current_bearing_ = imu_yaw;
            RCLCPP_INFO(this->get_logger(), "Updated Bearing from IMU: %.2f degrees", current_bearing_);
        }
    }



    void timer_callback() {
        if (!gps_initialized_ || std::isnan(target_bearing_)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for GPS & Target...");
            return;
        }

        // Normalize the error to range [-180, 180]
        double error = target_bearing_ - current_bearing_;
        while (error > 180.0) error -= 360.0;
        while (error < -180.0) error += 360.0;

        double distance_to_target = sqrt(pow(target_latitude_ - current_latitude_, 2) +
                                        pow(target_longitude_ - current_longitude_, 2));

        auto cmd_vel_msg = geometry_msgs::msg::Twist();

        if (std::abs(error) < 3.0 && distance_to_target < 0.00001) {  // Stop if close enough
            if (!has_stopped_) {  
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_msg.linear.x = 0.0;
                velocity_publisher_->publish(cmd_vel_msg);
                RCLCPP_INFO(this->get_logger(), "Target reached! Stopping rover.");
                has_stopped_ = true;
                target_bearing_ = std::numeric_limits<double>::quiet_NaN();  // Reset target
            }
        } else {
            has_stopped_ = false;
            cmd_vel_msg.angular.z = 0.5 * (error / 180.0);
            cmd_vel_msg.linear.x = std::max(0.05, 0.2 * (1.0 - std::abs(error) / 180.0));
            velocity_publisher_->publish(cmd_vel_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Error: %.2f | Distance: %.6f | Linear: %.2f | Angular: %.2f", 
                    error, distance_to_target, cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
    }


    bool gps_initialized_ = false;
    double reference_latitude_ = 0.0;
    double reference_longitude_ = 0.0;
    double current_latitude_ = 0.0;
    double current_longitude_ = 0.0;

    double target_latitude_ = std::numeric_limits<double>::quiet_NaN();
    double target_longitude_ = std::numeric_limits<double>::quiet_NaN();


    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_bearing_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;  
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_; // ✅ GPS Subscription
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    
    double current_bearing_ = 0.0;
    double target_bearing_ = std::numeric_limits<double>::quiet_NaN();  
    bool has_stopped_ = false;  
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverControlNode>());
    rclcpp::shutdown();
    return 0;
}
