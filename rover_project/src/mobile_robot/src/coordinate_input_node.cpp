#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <iostream>
#include <cmath>

class CoordinateInputNode : public rclcpp::Node
{
public:
    CoordinateInputNode() : Node("coordinate_input_node")
    {
        // Initialize publisher
        bearing_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_bearing", 10);
        RCLCPP_INFO(this->get_logger(), "Coordinate Input Node is running...");
        get_and_convert_coordinates();
    }

private:
    // Reference origin for conversion (adjust as needed)
    double reference_latitude_ = 0.0;
    double reference_longitude_ = 0.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bearing_publisher_;

    // Haversine formula to calculate bearing
    double to_radians(double degree)
    {
        return degree * M_PI / 180.0;
    }

    double calculate_bearing(double lat1, double lon1, double lat2, double lon2)
    {
        // Convert degrees to radians
        lat1 = to_radians(lat1);
        lon1 = to_radians(lon1);
        lat2 = to_radians(lat2);
        lon2 = to_radians(lon2);

        // Calculate the bearing using the Haversine formula
        double delta_lon = lon2 - lon1;
        double y = sin(delta_lon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);
        double bearing = atan2(y, x);

        // Convert the bearing from radians to degrees
        bearing = fmod((bearing * 180.0 / M_PI) + 360.0, 360.0); // Ensure the bearing is between 0 and 360

        return bearing;
    }

    void get_and_convert_coordinates()
    {
        while (rclcpp::ok())
        {
            double target_latitude, target_longitude;
            std::cout << "Enter target coordinates (latitude longitude): ";
            std::cin >> target_latitude >> target_longitude;

            if (std::cin.fail())
            {
                std::cin.clear(); // Clear the error flag
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
                std::cout << "Invalid input. Please enter valid latitude and longitude values." << std::endl;
                continue;
            }

            // Calculate the bearing from the reference point to the target coordinates
            double bearing = calculate_bearing(reference_latitude_, reference_longitude_, target_latitude, target_longitude);

            // Print the results
            std::cout << "Calculated Bearing: " << bearing << " degrees" << std::endl;

            // Publish the calculated bearing
            auto msg = std_msgs::msg::Float64();
            msg.data = bearing;
            bearing_publisher_->publish(msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateInputNode>());
    rclcpp::shutdown();
    return 0;
}
