#include "rclcpp/rclcpp.hpp" //ROS2 için C++ istemci kütüphanesi

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello cpp node!!!");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timerCallback, this));
        // create_wall_timer fonksiyonu bir shared pointer döndürür (rclcpp::TimerBase::SharedPtr)
        // eğer bu fonksiyonu bir değişkene eşitlemezsek fonksiyon bittiğinde shared pointerın referansı sıfır olur ve silinir. 
    }

private:
    void timerCallback()
    {
        counter_ ++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
