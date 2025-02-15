#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <GeographicLib/Geodesic.hpp>
#include <atomic>
#include <mutex>
#include <cmath>

class NavigationController : public rclcpp::Node {
public:
    NavigationController() : Node("navigation_controller") {
        // Declare parameters
        this->declare_parameter("gps_topic", "gps");
        this->declare_parameter("bearing_topic", "target_bearing");
        this->declare_parameter("max_retries", 3);
        this->declare_parameter("update_rate", 1.0);

        // Initialize services and publishers
        initialize_components();
        RCLCPP_INFO(this->get_logger(), "Navigation controller initialized");
    }

private:
    std::mutex gps_mutex_;
    std::atomic<bool> gps_initialized_{false};
    double current_latitude_ = 0.0;
    double current_longitude_ = 0.0;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bearing_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr input_service_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    void initialize_components() {
        // Create components with parameterized topics
        bearing_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            this->get_parameter("bearing_topic").as_string(), 10);

        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            this->get_parameter("gps_topic").as_string(), 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                handle_gps_update(msg);
            });

        input_service_ = this->create_service<std_srvs::srv::Trigger>(
            "request_coordinate_input",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                handle_coordinate_input(request, response);
            });

        // Initialize update timer
        update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / this->get_parameter("update_rate").as_double()),
            [this]() { update_navigation(); });
    }

    void handle_gps_update(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            std::lock_guard<std::mutex> lock(gps_mutex_);
            current_latitude_ = msg->latitude;
            current_longitude_ = msg->longitude;
            if (!gps_initialized_.exchange(true)) {
                RCLCPP_INFO(this->get_logger(), "GPS initialized with valid fix");
            }
        }
    }

    void handle_coordinate_input(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!gps_initialized_) {
            response->success = false;
            response->message = "GPS not initialized";
            return;
        }

        double target_lat, target_lon;
        RCLCPP_INFO(this->get_logger(), "Enter target coordinates (latitude longitude): ");
        
        int attempts = 0;
        while (rclcpp::ok() && attempts++ < this->get_parameter("max_retries").as_int()) {
            std::cin >> target_lat >> target_lon;
            
            if (validate_coordinates(target_lat, target_lon)) {
                calculate_and_publish_bearing(target_lat, target_lon);
                response->success = true;
                response->message = "Coordinates accepted";
                return;
            }
            
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(this->get_logger(), "Invalid coordinates. Please try again.");
        }

        response->success = false;
        response->message = "Maximum input attempts exceeded";
    }

    bool validate_coordinates(double lat, double lon) {
        return (lat >= -90.0 && lat <= 90.0) && (lon >= -180.0 && lon <= 180.0);
    }

    void calculate_and_publish_bearing(double target_lat, double target_lon) {
        std::lock_guard<std::mutex> lock(gps_mutex_);
        
        try {
            GeographicLib::Geodesic geod = GeographicLib::Geodesic::WGS84();
            double azi1, azi2;
            geod.Inverse(current_latitude_, current_longitude_,
                        target_lat, target_lon, azi1, azi2);

            auto bearing_msg = geometry_msgs::msg::Vector3Stamped();
            bearing_msg.header.stamp = this->now();
            bearing_msg.header.frame_id = "navigation";
            bearing_msg.vector.x = azi1;  // Bearing
            bearing_msg.vector.y = GeographicLib::Geodesic::WGS84().Inverse(
                current_latitude_, current_longitude_, target_lat, target_lon);  // Distance
            bearing_msg.vector.z = 0.0;  // Reserved for future use

            bearing_publisher_->publish(bearing_msg);
            log_navigation_info(target_lat, target_lon, azi1);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Geodesic calculation failed: %s", e.what());
        }
    }

    void log_navigation_info(double target_lat, double target_lon, double bearing) {
        RCLCPP_INFO(this->get_logger(), "Current Position: [%.6f, %.6f]", 
                   current_latitude_, current_longitude_);
        RCLCPP_INFO(this->get_logger(), "Target Position: [%.6f, %.6f]", 
                   target_lat, target_lon);
        RCLCPP_INFO(this->get_logger(), "Calculated Bearing: %.2f°", bearing);
    }

    void update_navigation() {
        if (!gps_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Waiting for GPS initialization...");
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}