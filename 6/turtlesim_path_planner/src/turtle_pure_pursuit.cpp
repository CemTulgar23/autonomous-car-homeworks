#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <cstdlib>
#include <ctime>
#include <cmath>

using namespace std;

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        // Başlangıç konumu ayarları
        this->declare_parameter("start_x", 1.0);
        this->declare_parameter("start_y", 1.0);
        this->get_parameter("start_x", start_x_);
        this->get_parameter("start_y", start_y_);

        // spawn_client = this->create_client<turtlesim::srv::Spawn>("/spawn");
        teleport_client = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
        pen_client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        spawn_client = this->create_client<turtlesim::srv::Spawn>("/spawn");

        // Turtlesim'e komut gönderecek olan publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Kaplumbağanın pozisyonunu alacak olan subscriber
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));

        // Başlangıç pozisyonunu spawn servisi ile sıfırlıyoruz
        setTurtleBeginnigPosition();

        spawnPointTurtle();

        // Hedefe doğru hareket et
        turtle_vel_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleController::move_to_target, this));

        new_point_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TurtleController::spawnPointTurtle, this));
    }

private:
    float generateRandomFloat(float min, float max)
    {
        // Rastgele sayı üretmek için "rand()" fonksiyonu kullanılır.
        float range = max - min;
        float random_value = min + (rand() / (RAND_MAX / range));
        return random_value;
    }

    void setTurtleBeginnigPosition()
    {
        while (!teleport_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /turtle1/teleport_absolute service to be available...");
        }

        while (!pen_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /turtle1/set_pen service to be available...");
        }

        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();

        // Kalemi kapat
        pen_request->off = 1;
        pen_client->async_send_request(pen_request);

        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = 1.0;     // X konumu
        request->y = 1.0;     // Y konumu
        request->theta = 0.0; // Başlangıç açısı

        // Teleport işlemini başlat ve tamamlanmasını bekle
        auto future = teleport_client->async_send_request(request);

        // Kalemi tekrar aç
        pen_request->off = 0;
        pen_client->async_send_request(pen_request);
    }

    void spawnPointTurtle()
    {
        while (!spawn_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /spawn service to be available...");
        }

        if (is_path_complated == true) // patika tamamlandıysa yeni kaplımbağa spawnla
        {
            srand(static_cast<unsigned int>(time(0)));

            goal_x = generateRandomFloat(0.5f, 10.5f);
            goal_y = generateRandomFloat(0.5f, 10.5f);

            RCLCPP_INFO(this->get_logger(), "goal_x: %f, goal_y: %f", goal_x, goal_y);

            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = goal_x;
            request->y = goal_y;
            request->theta = 0.0;

            static int counter_ = 2;
            request->name = "turtle" + std::to_string(counter_++);

            // Servisi çağırıyoruz
            auto result = spawn_client->async_send_request(request);

            is_path_complated = false; // artık yeni path oluştu, bu yüzden tamamlanma false oldu
        }
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;
    }

    void move_to_target()
    {
        geometry_msgs::msg::Twist move_cmd;

        // LookAhead mesafesini kullanarak yeni hedef noktası hesapla
        float look_ahead_x = current_x_ + look_ahead_distance_ * cos(current_theta_);
        float look_ahead_y = current_y_ + look_ahead_distance_ * sin(current_theta_);

        float angle_to_target = atan2(goal_y - look_ahead_y, goal_x - look_ahead_x); // robotun bulunduğu konum ile hedef konum arasındaki doğrunun eğimini buluyoruz
        static float last_angle_diff = 0;
        float angle_diff = angle_to_target - current_theta_; // doğrunun eğimi ile robotun eğimi arasındaki farkı buluyoruz, böylelikle robotun ne kadar dönmesi gerektiğini anlamış oluyoruz

        // RCLCPP_INFO(this->get_logger(), "angel_to_target: %f", angle_to_target);
        // RCLCPP_INFO(this->get_logger(), "current_theta: %f", current_theta_);
        RCLCPP_INFO(this->get_logger(), "ange_diff: %f", angle_diff);
        RCLCPP_INFO(this->get_logger(), "last_ange_diff: %f", last_angle_diff);

        // radyan cinsindeki açının esas ölçüsünü buluyoruz, bunu yapmazsak robot gereksiz yere birkaç daire çizebilir doğru yöne ulaşmak için
        if (angle_diff > 3.14)
            angle_diff -= 6.28;
        if (angle_diff < -3.14)
            angle_diff += 6.28;


        if (angle_diff * last_angle_diff >= 0)
        {
            move_cmd.angular.z = angle_diff * 1.0;
            last_angle_diff = angle_diff;
        }
        else
        {
            move_cmd.angular.z = 0.0;
        }

        float distance = sqrt(pow(goal_x - current_x_, 2) + pow(goal_y - current_y_, 2));

        if (distance < 0.1) // Hedefe yaklaşınca dur
        {
            RCLCPP_INFO(this->get_logger(), "Reached target.");
            move_cmd.linear.x = 0.0; // İlerleme oranı
            last_angle_diff = 0;
            is_path_complated = true;
        }
        else
        {
            move_cmd.linear.x = distance * 0.5; // İlerleme oranı
        }

        // Hedefe doğru hareket et
        cmd_vel_pub_->publish(move_cmd);

        // Küçük bir süre bekleyip tekrar kontrol et
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

private:
    rclcpp::TimerBase::SharedPtr turtle_vel_timer;
    rclcpp::TimerBase::SharedPtr new_point_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client;

    float start_x_, start_y_;
    float current_x_, current_y_, current_theta_;
    float goal_x, goal_y;
    float look_ahead_distance_ = 1.0;

    bool initial_pose_set = false; // İlk konumun kaydedildiğini takip etmek için
    bool is_path_complated = true;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
