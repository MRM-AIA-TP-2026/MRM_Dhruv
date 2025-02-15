#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <chrono>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

class GPSMonitor : public rclcpp::Node {
public:
    GPSMonitor() : Node("gps_monitor") {
        // Declare parameters
        this->declare_parameter("gps_topic", "gps");
        this->declare_parameter("update_interval", 2.0);
        this->declare_parameter("log_throttle_seconds", 5);
        this->declare_parameter("warn_stale_seconds", 10);

        // Initialize components
        setup_subscription();
        setup_timer();
        
        RCLCPP_INFO(this->get_logger(), "GPS Monitor Node initialized");
    }

private:
    struct GPSCoordinates {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        rclcpp::Time stamp;
        int status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        uint position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    };

    std::mutex data_mutex_;
    GPSCoordinates current_fix_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    std::atomic<bool> gps_available_{false};

    void setup_subscription() {
        auto qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            this->get_parameter("gps_topic").as_string(),
            qos,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                handle_gps_update(msg);
            });
    }

    void setup_timer() {
        const double interval = this->get_parameter("update_interval").as_double();
        monitor_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(interval),
            [this]() { monitor_gps_status(); });
    }

    void handle_gps_update(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        try {
            if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
                current_fix_.latitude = msg->latitude;
                current_fix_.longitude = msg->longitude;
                current_fix_.altitude = msg->altitude;
                current_fix_.stamp = msg->header.stamp;
                current_fix_.status = msg->status.status;
                current_fix_.position_covariance_type = msg->position_covariance_type;
                gps_available_.store(true);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 
                                   this->get_parameter("log_throttle_seconds").as_int() * 1000,
                                   "GPS fix lost or degraded");
                gps_available_.store(false);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing GPS data: %s", e.what());
        }
    }

    void monitor_gps_status() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto now = this->now();
        const auto stale_threshold = this->get_parameter("warn_stale_seconds").as_int();

        if (!gps_available_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for GPS signal...");
            return;
        }

        if ((now - current_fix_.stamp).seconds() > stale_threshold) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 
                               this->get_parameter("log_throttle_seconds").as_int() * 1000,
                               "Stale GPS data: %.1f seconds old", 
                               (now - current_fix_.stamp).seconds());
            return;
        }

        std::string fix_type;
        switch(current_fix_.status) {
            case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
                fix_type = "SBAS"; break;
            case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
                fix_type = "GBAS"; break;
            case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
                fix_type = "Standard"; break;
            default:
                fix_type = "Unknown"; break;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 
                           this->get_parameter("log_throttle_seconds").as_int() * 1000,
                           "GPS Fix [%s]:\n- Latitude: %.7f\n- Longitude: %.7f\n- Altitude: %.2f\n- Covariance: %d",
                           fix_type.c_str(),
                           current_fix_.latitude,
                           current_fix_.longitude,
                           current_fix_.altitude,
                           current_fix_.position_covariance_type);
    }
};

int main(int argc char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSMonitor>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}