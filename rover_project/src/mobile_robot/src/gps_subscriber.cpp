#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber() : Node("gps_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));

        // Set up the timer to call gps_callback every 5 seconds
        timer_ = this->create_wall_timer(
            5s, std::bind(&GPSSubscriber::print_gps_data, this));
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Store the most recent GPS data
        latitude_ = msg->latitude;
        longitude_ = msg->longitude;
        altitude_ = msg->altitude;
    }

    void print_gps_data()
    {
        // Print the latest GPS data every 5 seconds
        RCLCPP_INFO(this->get_logger(), "GPS Position:");
        RCLCPP_INFO(this->get_logger(), "Latitude: %.7f", latitude_);
        RCLCPP_INFO(this->get_logger(), "Longitude: %.7f", longitude_);
        RCLCPP_INFO(this->get_logger(), "Altitude: %.2f", altitude_);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double latitude_, longitude_, altitude_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
