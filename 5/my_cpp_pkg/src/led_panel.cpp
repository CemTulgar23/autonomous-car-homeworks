#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::placeholders;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel"), led_states_(3, 0)
    {
        led_status_publisher_ =
            this->create_publisher<my_robot_interfaces::msg::LedPanelState>(
                "led_panel_state", 10);
        led_status_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&LedPanelNode::publishLedStates, this));
        set_led_service_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Led panel node has been started");
    }

private:
    void publishLedStates()
    {
        auto msg = my_robot_interfaces::msg::LedPanelState();
        msg.led_states = led_states_;
        led_status_publisher_->publish(msg);
    }

    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        int64_t led_number = request->led_number;
        int64_t led_status = request->led_status;

        if (led_number >= (int64_t)led_states_.size() || led_number < 0)
        {
            response->success = false;
            return;
        }

        if (led_status != 0 && led_status != 1)
        {
            response->success = false;
            return;
        }

        led_states_.at(led_number) = led_status;

        for (size_t i = 0; i < led_states_.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "%zu. LED state: %ld", i, led_states_[i]);
        }
        
        response->success = true;
        publishLedStates();
    }

    std::vector<int64_t> led_states_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr led_status_publisher_;
    rclcpp::TimerBase::SharedPtr led_status_timer_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr set_led_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}