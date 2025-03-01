#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_srvs/srv/empty.hpp"

class JoystickNode : public rclcpp::Node
{
public:
    JoystickNode() : Node("joystick_reader")
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickNode::joy_callback, this, std::placeholders::_1));

        turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&JoystickNode::turtle_pose_callback, this, std::placeholders::_1));

        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        clear_background_client_ = this->create_client<std_srvs::srv::Empty>("/clear");
        reset_client_ = this->create_client<std_srvs::srv::Empty>("/reset"); 
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn"); 

        RCLCPP_INFO(this->get_logger(), "Joystick Reader Node Started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        static bool is_clicked = false;

        geometry_msgs::msg::Twist twist_msg;

        float max_speed = 2.0;      // Kaplumbağanın maksimum doğrusal hızı
        float max_turn_speed = 2.0; // Kaplumbağanın maksimum dönüş hızı

        // İleri-Geri Hareket (axes[7])
        twist_msg.linear.x = msg->axes[7] * max_speed;

        // Sağ-Sol Dönüş (axes[4])
        twist_msg.angular.z = msg->axes[4] * max_turn_speed;

        // Mesajı cmd_vel konusuna yayınla
        cmd_vel_publisher_->publish(twist_msg);

        // Button 0 (A tuşu) basıldı mı?
        if (msg->buttons[0] == 1 && is_clicked == false)
        {
            is_clicked = true;
            clear_trail();
        }

        // Button 1 (B tuşu) basıldı mı? Eğer evet, turtlesim'i resetle
        if (msg->buttons[1] == 1 && is_clicked == false)
        {
            is_clicked = true;
            reset_turtlesim();
        }

        // Button 2 (X tuşu) basıldı mı? Spawn için
        if (msg->buttons[2] == 1 && is_clicked == false)
        {
            spawn_turtle();
        }

        // Button 3 (Y tuşu) basıldı mı? Spawn için
        if (msg->buttons[3] == 1 && is_clicked == false)
        {
            set_pen();
        }

        is_clicked = msg->buttons[2];
    }

    void turtle_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
    }

    void clear_trail()
    {
        if (!set_pen_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "set_pen servisi bulunamadı!");
            return;
        }

        // 1. Çizgiyi kapat (off = 1)
        auto request_off = std::make_shared<turtlesim::srv::SetPen::Request>();
        request_off->off = 1;
        set_pen_client_->async_send_request(request_off);

        // 2. Arka planı temizle
        if (clear_background_client_->wait_for_service(std::chrono::seconds(1)))
        {
            auto request_clear = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_background_client_->async_send_request(request_clear);
        }

        // 3. Çizgiyi tekrar aç (off = 0)
        auto request_on = std::make_shared<turtlesim::srv::SetPen::Request>();
        request_on->off = 0;
        set_pen_client_->async_send_request(request_on);

        RCLCPP_INFO(this->get_logger(), "Çizgi silindi!");
    }

    void reset_turtlesim()
    {
        if (!reset_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "/reset servisi bulunamadı!");
            return;
        }

        // Reset servisini çağır
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        reset_client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Turtlesim resetlendi!");
    }

    void spawn_turtle()
    {
        // Yeni kaplumbağa oluşturma servisini çağırma
        if (!spawn_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "spawn servisi bulunamadı!");
            return;
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        // Mevcut kaplumbağanın pozisyonunu almak için, "/turtle1/pose" konusuna abone olunabilir veya sabit bir değer kullanılabilir.
        request->x = x;         
        request->y = y;      
        request->theta = theta; 

        static int counter = 2;
        request->name = std::string("turtle") + std::to_string(counter++);

        // Yeni kaplumbağa oluşturma isteğini gönder
        spawn_client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Yeni kaplumbağa spawn edildi!");
    }

    void set_pen()
    {
        auto client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "SetPen servisi bekleniyor...");
        }

        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();

        request->r = 255;              // Kırmızı renk
        request->g = 0;                // Yeşil yok
        request->b = 0;                // Mavi yok
        request->width = 3;            // Kalem kalınlığı

        auto future = client->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Color has been changed.");
    }

    float x;
    float y;
    float theta;

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_background_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
