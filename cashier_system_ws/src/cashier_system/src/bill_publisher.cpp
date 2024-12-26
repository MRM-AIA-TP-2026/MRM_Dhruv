#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"

using namespace std::chrono_literals;

class BillPublisher : public rclcpp::Node
{
public:
    BillPublisher() : Node("bill_publisher"), bill_number_(1)
    {
        publisher_ = this->create_publisher<cashier_system::msg::Bill>("bill", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&BillPublisher::publish_bill, this));
    }

private:
    void publish_bill()
    {
        auto message = cashier_system::msg::Bill();
        message.bill_number = bill_number_++;
        message.time_stamp = this->get_clock()->now();
        message.quantity = 5; // Example quantity
        message.price = 20.0; // Example price
        message.total = message.quantity * message.price;

        RCLCPP_INFO(this->get_logger(), "Publishing Bill: %d", message.bill_number);
        publisher_->publish(message);
    }

    rclcpp::Publisher<cashier_system::msg::Bill>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int bill_number_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BillPublisher>());
    rclcpp::shutdown();
    return 0;
}
