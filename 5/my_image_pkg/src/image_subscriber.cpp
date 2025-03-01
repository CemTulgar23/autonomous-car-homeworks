#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("image_subscriber")
    {
        blur_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "blur_image", 10,
            std::bind(&ImageSubscriberNode::blur_image_callback, this, std::placeholders::_1));
        
        grayscale_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "grayscale_image", 10,
            std::bind(&ImageSubscriberNode::grayscale_image_callback, this, std::placeholders::_1));
        
        cany_edge_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "cany_edge_image", 10,
            std::bind(&ImageSubscriberNode::cany_edge_image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "image subscriber has been started.");
    }

private:
    void blur_image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Blur image callback triggered.");
        cv::Mat frame1 = cv_bridge::toCvCopy(msg, "bgr8")->image; // sensor_msgs::msgs::Image tipindeki veriyi cv::Mat tipindeki değişkene çeviriyoruz
        cv::imshow("Video Stream Blur", frame1);                       // videoyu ayrı bir pencerede açıyoruz
        
        cv::waitKey(1);                                          // 1 milisaniye bekle
    }

    void grayscale_image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Grayscale image callback triggered.");
        cv::Mat frame2 = cv_bridge::toCvCopy(msg, "mono8")->image; // sensor_msgs::msgs::Image tipindeki veriyi cv::Mat tipindeki değişkene çeviriyoruz
        cv::imshow("Video Stream Grayscale", frame2);                       // videoyu ayrı bir pencerede açıyoruz
        
        cv::waitKey(1);                                          // 1 milisaniye bekle
    }

    void cany_edge_image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Cany Edge image callback triggered.");
        cv::Mat frame3 = cv_bridge::toCvCopy(msg, "mono8")->image; // sensor_msgs::msgs::Image tipindeki veriyi cv::Mat tipindeki değişkene çeviriyoruz
        cv::imshow("Video Stream Cany Edge", frame3);                       // videoyu ayrı bir pencerede açıyoruz
        
        cv::waitKey(1);                                          // 1 milisaniye bekle
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr blur_image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr grayscale_image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cany_edge_image_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}