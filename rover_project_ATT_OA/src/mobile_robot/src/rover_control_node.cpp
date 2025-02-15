#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "GeographicLib/Geodesic.hpp"
#include <chrono>
#include <mutex>
#include <atomic>
#include <limits>

using namespace std::chrono_literals;

class RoverController : public rclcpp::Node {
public:
    RoverController() : Node("rover_controller") {
        declare_parameters();
        initialize_components();
        RCLCPP_INFO(this->get_logger(), "Rover Controller initialized");
    }

private:
    struct NavigationState {
        double current_bearing = 0.0;
        double target_bearing = std::numeric_limits<double>::quiet_NaN();
        double current_latitude = 0.0;
        double current_longitude = 0.0;
        double target_latitude = std::numeric_limits<double>::quiet_NaN();
        double target_longitude = std::numeric_limits<double>::quiet_NaN();
        bool gps_initialized = false;
        bool has_stopped = false;
    };

    void declare_parameters() {
        this->declare_parameter("kp_angular", 0.8);
        this->declare_parameter("ki_angular", 0.01);
        this->declare_parameter("kd_angular", 0.1);
        this->declare_parameter("max_linear", 0.3);
        this->declare_parameter("min_linear", 0.1);
        this->declare_parameter("stop_distance", 1.0);
        this->declare_parameter("stop_angle", 2.0);
        this->declare_parameter("gps_noise_threshold", 0.00001);
    }

    void initialize_components() {
        auto qos = rclcpp::SensorDataQoS();
        
        // Subscriptions
        bearing_sub_ = create_subscription<std_msgs::msg::Float64>(
            "target_bearing", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_.target_bearing = msg->data;
            });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                update_orientation(msg);
            });

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", qos, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                update_position(msg);
            });

        // Publisher
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Control timer
        control_timer_ = create_wall_timer(
            100ms, std::bind(&RoverController::control_loop, this));
    }

    void update_orientation(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.current_bearing = normalize_angle(yaw * (180.0 / M_PI));
    }

    void update_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (!state_.gps_initialized && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            state_.current_latitude = msg->latitude;
            state_.current_longitude = msg->longitude;
            state_.gps_initialized = true;
            RCLCPP_INFO(get_logger(), "GPS initialized");
        }

        const double noise_thresh = get_parameter("gps_noise_threshold").as_double();
        if (std::abs(msg->latitude - state_.current_latitude) > noise_thresh ||
            std::abs(msg->longitude - state_.current_longitude) > noise_thresh) {
            state_.current_latitude = msg->latitude;
            state_.current_longitude = msg->longitude;
        }
    }

    void control_loop() {
        NavigationState local_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            local_state = state_;
        }

        if (!validate_state(local_state)) return;

        const auto [error, distance] = calculate_navigation_errors(local_state);
        auto cmd_vel = calculate_velocity_commands(error, distance);

        publish_velocity(cmd_vel);
        log_state(local_state, error, distance);
    }

    bool validate_state(const NavigationState& state) {
        if (!state.gps_initialized) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for GPS fix...");
            return false;
        }
        
        if (std::isnan(state.target_bearing)) {
            RCLCPP_DEBUG(get_logger(), "No target bearing set");
            return false;
        }
        
        return true;
    }

    std::pair<double, double> calculate_navigation_errors(const NavigationState& state) {
        double error = normalize_angle(state.target_bearing - state.current_bearing);
        
        GeographicLib::Geodesic geod = GeographicLib::Geodesic::WGS84();
        double distance;
        geod.Inverse(state.current_latitude, state.current_longitude,
                    state.target_latitude, state.target_longitude, distance);

        return {error, distance};
    }

    geometry_msgs::msg::Twist calculate_velocity_commands(double error, double distance) {
        geometry_msgs::msg::Twist cmd_vel;
        const double stop_dist = get_parameter("stop_distance").as_double();
        const double stop_angle = get_parameter("stop_angle").as_double();

        if (distance < stop_dist && std::abs(error) < stop_angle) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            state_.has_stopped = true;
        } else {
            cmd_vel.angular.z = pid_control(error);
            cmd_vel.linear.x = std::clamp(
                get_parameter("max_linear").as_double() * 
                (1.0 - std::abs(error)/180.0),
                get_parameter("min_linear").as_double(),
                get_parameter("max_linear").as_double()
            );
            state_.has_stopped = false;
        }
        return cmd_vel;
    }

    double pid_control(double error) {
        static double integral = 0.0;
        static double prev_error = 0.0;
        const double dt = 0.1;  // Control period

        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;

        return get_parameter("kp_angular").as_double() * error +
               get_parameter("ki_angular").as_double() * integral +
               get_parameter("kd_angular").as_double() * derivative;
    }

    double normalize_angle(double angle) {
        angle = fmod(angle + 180.0, 360.0);
        return angle >= 0 ? angle - 180.0 : angle + 180.0;
    }

    void publish_velocity(const geometry_msgs::msg::Twist& cmd_vel) {
        cmd_vel_pub_->publish(cmd_vel);
    }

    void log_state(const NavigationState& state, double error, double distance) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Bearing: %.1f° | Target: %.1f° | Error: %.1f° | Distance: %.2fm | Speed: %.2f",
            state.current_bearing, state.target_bearing, error, distance,
            state.has_stopped ? 0.0 : get_parameter("max_linear").as_double());
    }

    std::mutex state_mutex_;
    NavigationState state_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bearing_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}