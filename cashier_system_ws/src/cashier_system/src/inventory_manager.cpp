#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/time.hpp"

class InventoryManager : public rclcpp::Node
{
public:
    InventoryManager() : Node("inventory_manager")
    {
        this->declare_parameter<int>("inventory", 100);
        this->declare_parameter<float>("income", 0.0);

        subscriber_ = this->create_subscription<cashier_system::msg::Bill>(
            "bill", 10, std::bind(&InventoryManager::process_bill, this, std::placeholders::_1));

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "update_parameters", std::bind(&InventoryManager::update_parameters, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Declare member variables
    rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscriber_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    void process_bill(const cashier_system::msg::Bill::SharedPtr msg)
    {
        auto inventory = this->get_parameter("inventory").as_int();
        auto income = this->get_parameter("income").as_double();

        inventory -= msg->quantity;
        income += msg->total;

        RCLCPP_INFO(this->get_logger(), "Processed Bill: %u, New Inventory: %ld, New Income: %.2f", msg->bill_number, inventory, income);

        this->set_parameter(rclcpp::Parameter("inventory", inventory));
        this->set_parameter(rclcpp::Parameter("income", income));
    }

    void update_parameters(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            this->set_parameter(rclcpp::Parameter("inventory", 100));
            this->set_parameter(rclcpp::Parameter("income", 0.0));
            response->success = true;
            response->message = "Parameters reset.";
        }
        else
        {
            response->success = false;
            response->message = "No action taken.";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InventoryManager>());
    rclcpp::shutdown();
    return 0;
}
