#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <iostream>
#include <cmath>

class CoordinateInputNode : public rclcpp::Node {
public:
    CoordinateInputNode() : Node("coordinate_input_node"), gps_initialized_(false) {
        // Initialize publisher
        bearing_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_bearing", 10);
        
        // Subscribe to GPS position
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&CoordinateInputNode::gps_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Coordinate Input Node initialized");

        get_and_convert_coordinates();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bearing_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    double current_latitude_ = 0.0;
    double current_longitude_ = 0.0;
    double reference_latitude_ = 0.0;
    double reference_longitude_ = 0.0;
    bool gps_initialized_ = false;

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!gps_initialized_ && std::abs(msg->latitude) > 1e-6 && std::abs(msg->longitude) > 1e-6) {
        reference_latitude_ = msg->latitude;
        reference_longitude_ = msg->longitude;
        gps_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Reference Position Set: Lat: %.6f, Lon: %.6f", 
                    reference_latitude_, reference_longitude_);
    }

    current_latitude_ = msg->latitude;
    current_longitude_ = msg->longitude;
}


    double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
        const double DEG_TO_RAD = M_PI / 180.0;
        const double RAD_TO_DEG = 180.0 / M_PI;
        
        lat1 *= DEG_TO_RAD;
        lon1 *= DEG_TO_RAD;
        lat2 *= DEG_TO_RAD;
        lon2 *= DEG_TO_RAD;

        double delta_lon = lon2 - lon1;
        double y = sin(delta_lon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);
        
        double bearing = atan2(y, x) * RAD_TO_DEG;
        return fmod(bearing + 360.0, 360.0); // Normalize to [0,360]
    }

    void get_and_convert_coordinates() {
        while (rclcpp::ok()) {
            double target_latitude, target_longitude;
            std::cout << "Enter target coordinates (latitude longitude): ";
            std::cin >> target_latitude >> target_longitude;

            if (std::cin.fail()) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                RCLCPP_ERROR(this->get_logger(), "Invalid input. Please enter valid coordinates.");
                continue;
            }

            double bearing = calculate_bearing(current_latitude_, current_longitude_, target_latitude, target_longitude);

            RCLCPP_INFO(this->get_logger(), 
                        "Current: Lat: %.6f, Lon: %.6f", current_latitude_, current_longitude_);
            RCLCPP_INFO(this->get_logger(), 
                        "Target: Lat: %.6f, Lon: %.6f", target_latitude, target_longitude);
            RCLCPP_INFO(this->get_logger(), "Bearing: %.2f degrees", bearing);

            auto msg = std_msgs::msg::Float64();
            msg.data = bearing;
            bearing_publisher_->publish(msg);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateInputNode>());
    rclcpp::shutdown();
    return 0;
}
