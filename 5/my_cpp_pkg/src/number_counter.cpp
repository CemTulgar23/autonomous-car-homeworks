#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number",
            10,
            std::bind(&NumberCounterNode::getNumber, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberCounterNode::publishNumber, this));
        service_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_number_count",
            std::bind(&NumberCounterNode::resetNumberCount, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "number counter has been started.");
    }

private:
    void getNumber(const example_interfaces::msg::Int64 &msg)
    {
        number_.data = msg.data;
    }

    void publishNumber()
    {
        publisher_->publish(number_);
    }

    void resetNumberCount(
        const example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data == true) // Gelen bool değeri kontrol ediliyor
        {
            number_.data = 0;
            response->success = true; // İşlem başarılı
            response->message = "Number count reset successfully.";
        }
        else
        {
            response->success = false; // İşlem başarısız
            response->message = "Reset condition was false.";
        }
    }

    example_interfaces::msg::Int64 number_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}