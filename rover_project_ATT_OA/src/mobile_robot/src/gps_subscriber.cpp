#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <chrono>

using namespace std::chrono_literals;

class GPSSubscriber : public rclcpp::Node {
public:
    GPSSubscriber() : Node("gps_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            5s, std::bind(&GPSSubscriber::print_gps_data, this));
            
        RCLCPP_INFO(this->get_logger(), "GPS Subscriber Node initialized");
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        latitude_ = msg->latitude;
        longitude_ = msg->longitude;
        altitude_ = msg->altitude;
    }

    void print_gps_data() {
        RCLCPP_INFO(this->get_logger(), "GPS Position: Latitude: %.7f, Longitude: %.7f, Altitude: %.2f",
                    latitude_, longitude_, altitude_);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double latitude_ = 0.0;
    double longitude_ = 0.0;
    double altitude_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
