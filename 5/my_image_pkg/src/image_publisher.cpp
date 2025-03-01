#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode() : Node("image_publisher")
    {
        this->declare_parameter<std::string>("video_file", "");
        this->declare_parameter<int>("frame_rate", 30);
        this->declare_parameter("filter_params.gaussian_blur.kernel_size", 5);
        this->declare_parameter("filter_params.gaussian_blur.sigma", 1.0);
        this->declare_parameter("filter_params.canny_edge_detection.threshold1", 100);
        this->declare_parameter("filter_params.canny_edge_detection.threshold2", 200);

        std::string video_path = this->get_parameter("video_file").as_string();
        
        if (video_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Video file path is empty.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Video file path: %s", video_path.c_str());
        }
        
        int frame_rate = this->get_parameter("frame_rate").as_int();
        this->get_parameter("filter_params.gaussian_blur.kernel_size", kernel_size_);
        this->get_parameter("filter_params.gaussian_blur.sigma", sigma_);
        this->get_parameter("filter_params.canny_edge_detection.threshold1", threshold1_);
        this->get_parameter("filter_params.canny_edge_detection.threshold2", threshold2_);

        cap.open(video_path);
        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Video file is opening: %s", video_path.c_str());
            return;
        }

        blur_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "blur_image", 10);

        grayscale_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "grayscale_image", 10);

        cany_edge_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "cany_edge_image", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / frame_rate),
            std::bind(&ImagePublisherNode::publish_images, this));

        RCLCPP_INFO(this->get_logger(), "image publisher has been started.");
    }

private:
    void publish_images()
    {
        cv::Mat frame;
        if (!cap.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "Video bitti. Tekrar başlatiliyor...");
            cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Videoyu başa sar
            return;
        }

        // Blur Frame
        cv::Mat blurred_frame;
        cv::GaussianBlur(frame, blurred_frame, cv::Size(kernel_size_, kernel_size_), sigma_);
        auto blur_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blurred_frame).toImageMsg(); // cvv::Mat tipindeki veriyi sensor_msgs::msgs::Image tipindeki değişkene çeviriyoruz
        blur_image_publisher_->publish(*blur_msg);

        // Grayscale Frame
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        auto gray_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_frame).toImageMsg(); // cvv::Mat tipindeki veriyi sensor_msgs::msgs::Image tipindeki değişkene çeviriyoruz
        grayscale_image_publisher_->publish(*gray_msg);

        // Cany Edge Frame
        cv::Mat cany_edge_frame;
        cv::Canny(frame, cany_edge_frame, 100, 200);
        auto cany_edge_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cany_edge_frame).toImageMsg(); // cvv::Mat tipindeki veriyi sensor_msgs::msgs::Image tipindeki değişkene çeviriyoruz
        cany_edge_image_publisher_->publish(*cany_edge_msg);
    }

    int kernel_size_;
    double sigma_;
    int threshold1_;
    int threshold2_;

    cv::VideoCapture cap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr blur_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grayscale_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cany_edge_image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}