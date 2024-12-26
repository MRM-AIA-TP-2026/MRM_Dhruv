#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"

class LastBillPrinter : public rclcpp::Node
{
public:
    LastBillPrinter() : Node("last_bill_printer")
    {
        this->declare_parameter<int>("inventory", 100);
        this->declare_parameter<float>("income", 0.0);

        subscription_ = this->create_subscription<cashier_system::msg::Bill>(
            "bill", 10, std::bind(&LastBillPrinter::update_last_bill, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Last Bill Printer Node Initialized");
    }

    void print_status()
    {
        RCLCPP_INFO(this->get_logger(), "----- Current Status -----");
        RCLCPP_INFO(this->get_logger(), "Last Bill:");
        RCLCPP_INFO(this->get_logger(), "  Bill Number: %d", last_bill_.bill_number);
        RCLCPP_INFO(this->get_logger(), "  Timestamp: %d.%u", last_bill_.time_stamp.sec, last_bill_.time_stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "  Quantity: %d", last_bill_.quantity);
        RCLCPP_INFO(this->get_logger(), "  Price: %.2f", last_bill_.price);
        RCLCPP_INFO(this->get_logger(), "  Total: %.2f", last_bill_.total);

        auto inventory = this->get_parameter("inventory").as_int();
        auto income = this->get_parameter("income").as_double();
        RCLCPP_INFO(this->get_logger(), "Inventory: %ld", inventory);
        RCLCPP_INFO(this->get_logger(), "Income: %.2f", income);
    }

private:
    void update_last_bill(const cashier_system::msg::Bill::SharedPtr msg)
    {
        last_bill_ = *msg;
    }

    cashier_system::msg::Bill last_bill_;
    rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LastBillPrinter>();

    while (rclcpp::ok())
    {
        node->print_status();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    rclcpp::shutdown();
    return 0;
}
