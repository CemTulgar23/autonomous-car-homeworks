#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "std_srvs/srv/empty.hpp"
#include <cmath>

class TurtleNode : public rclcpp::Node
{
public:
    TurtleNode() : Node("turtle")
    {
        spawn_client = this->create_client<turtlesim::srv::Spawn>("/spawn");
        teleport_client = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
        // clear_background_client_ = this->create_client<std_srvs::srv::Empty>("/clear");
        pen_client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        turtle_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        setTurtleBeginnigPosition();

        /*
        // Başlangıçta 2 saniye beklemek için bir zamanlayıcı
        start_timer = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]()
            {
                startAfterDelay();
                start_timer.reset(); // Timer'ı sıfırla, böylece tekrar çalışmaz
            });
        */

        spawnPointTurtle();

        turtle_pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&TurtleNode::getTurtlePose, this, std::placeholders::_1));

        turtle_vel_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleNode::driveTurtle, this));

        new_point_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleNode::spawnPointTurtle, this));

        RCLCPP_INFO(this->get_logger(), "TurtleNode started.");
    }

    void startAfterDelay()
    {
        RCLCPP_INFO(this->get_logger(), "2 seconds passed, starting turtle setup...");
        spawnPointTurtle(); // Başlatma işlemini burada yapıyoruz
    }

    float generateRandomFloat(float min, float max)
    {
        // Rastgele sayı üretmek için "rand()" fonksiyonu kullanılır.
        float range = max - min;
        float random_value = min + (rand() / (RAND_MAX / range));
        return random_value;
    }

    void spawnPointTurtle()
    {
        while (!spawn_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /spawn service to be available...");
        }

        RCLCPP_INFO(this->get_logger(), "%i", is_path_complated);
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

            RCLCPP_INFO(this->get_logger(), "request_x: %f, request_y: %f, request_name: %s", request->x, request->y, request->name.c_str());

            // Servisi çağırıyoruz
            auto result = spawn_client->async_send_request(request);

            is_path_complated = false; // artık yeni path oluştu, bu yüzden tamamlanma false oldu
        }
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

        /*
        while (!clear_background_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /clear service to be available...");
        }
        */

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

        /*
        if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), "Turtle teleported, clearing background...");

            // Çizgileri temizleme servisini çağır
            auto request_clear = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_background_client_->async_send_request(request_clear);
        }
        */

        // Kalemi tekrar aç
        pen_request->off = 0;
        pen_client->async_send_request(pen_request);
    }

    void getTurtlePose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        if (!initial_pose_set)
        {
            initial_x = msg->x;
            initial_y = msg->y;
            initial_theta = msg->theta;
            initial_pose_set = true; // başlangıç pozisyonu atandı

            calculatePathSlope();

            RCLCPP_INFO(this->get_logger(), "Initial Pose -> X: %.2f, Y: %.2f, Theta: %.2f", initial_x, initial_y, initial_theta);
        }

        RCLCPP_INFO(rclcpp::get_logger("PoseSubscriber"), "X: %.2f, Y: %.2f, Theta: %.2f", msg->x, msg->y, msg->theta);
        turtle_x = msg->x;
        turtle_y = msg->y;
        turtle_theta = msg->theta;

        if (std::fabs(turtle_x - goal_x) < epsilon_ && std::fabs(turtle_y - goal_y) < epsilon_)
        {
            is_path_complated = true;
        }
    }

    void stopTurtle()
    {
        double distance = std::sqrt(std::pow(turtle_x - goal_x, 2) + std::pow(turtle_y - goal_y, 2));

        if (distance < epsilon_)
        { // Eğer belirlenen tolerans içinde ise durdur
            RCLCPP_INFO(this->get_logger(), "Hedefe ulaşildi, kaplumbağa duruyor!");

            // Hareketi durdur
            auto stop_msg = geometry_msgs::msg::Twist();
            turtle_vel_pub->publish(stop_msg);
            is_path_complated = true; // patika tamamlandı
            initial_pose_set = false; // yeni bir başlangıç noktası atanabilir artık

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    float calculatePathSlope()
    {

        if (goal_x == initial_x)
        {
            RCLCPP_WARN(this->get_logger(), "Slope is undefined (division by zero). Returning 0.");
            return 0;
        }

        path_slope = (goal_y - initial_y) / (goal_x - initial_x);
        float slope_radian = atan(path_slope);
        RCLCPP_INFO(this->get_logger(), "goal_x: %f, goal_y: %f, initial_x: %f, initial_y: %f, slope: %f", goal_x, goal_y, initial_x, initial_y, path_slope);
        RCLCPP_INFO(this->get_logger(), "slope radian: %f", slope_radian);
        return path_slope;
    }

    float calculateCrossTrackError()
    {
        // find path equation coefficients,
        float A = path_slope;
        float B = -1;
        float C = initial_y - path_slope * initial_x;

        // calculate distance between point and equation
        float cross_track_error = (A * turtle_x + B * turtle_y + C) / std::sqrt(A * A + B * B);
        RCLCPP_INFO(this->get_logger(), "A: %f, B: %f, C: %f, Cross Track Error: %f", A, B, C, cross_track_error);
        return cross_track_error;
    }

    void driveTurtle()
    {
        if (is_path_complated == false)
        {
            float cross_track_error = calculateCrossTrackError();

            // calculate difference between two slopes
            float slope_difference = turtle_theta - atan(path_slope);

            RCLCPP_INFO(this->get_logger(), "Cross Track Error: %.10f", cross_track_error);
            RCLCPP_INFO(this->get_logger(), "Slope Difference: %.10f", slope_difference);

            float angularity_coefficients = 10; // this will change if necessery.
            float speed = 1;
            geometry_msgs::msg::Twist twist;

            const float EPSILON = 0.001; // Küçük bir hata payı

            if (cross_track_error < 0) // çizginin solundaysa
            {
                if (cross_track_error != 0 && fabs(slope_difference) < EPSILON)
                {
                    twist.angular.z = angularity_coefficients * cross_track_error;
                }

                else if (cross_track_error != 0 && (fabs(slope_difference) > EPSILON && slope_difference > 0))
                {
                    twist.angular.z = -angularity_coefficients * fabs(slope_difference);
                }

                else if (cross_track_error != 0 && (fabs(slope_difference) > EPSILON && slope_difference < 0))
                {
                    twist.angular.z = -angularity_coefficients * fabs(slope_difference);
                }
            }
            else if (cross_track_error > 0) // çizginin sağındaysa
            {
                if (cross_track_error != 0 && fabs(slope_difference) < EPSILON)
                {
                    twist.angular.z = +angularity_coefficients * cross_track_error / 10;
                }

                else if (cross_track_error != 0 && (fabs(slope_difference) > EPSILON && slope_difference > 0))
                {
                    twist.angular.z = angularity_coefficients * fabs(slope_difference);
                }

                else if (cross_track_error != 0 && (fabs(slope_difference) > EPSILON && slope_difference < 0))
                {
                    twist.angular.z = angularity_coefficients * fabs(slope_difference);
                }
            }

            RCLCPP_INFO(this->get_logger(), "angular.z: %f", twist.angular.z);
            twist.linear.x = speed;
            turtle_vel_pub->publish(twist);
        }
        else if (is_path_complated == true)
        {
            stopTurtle();
        }
    }

private:
    float turtle_x;
    float turtle_y;
    float turtle_theta;

    float initial_x;     // Başlangıç x değeri (sabit)
    float initial_y;     // Başlangıç y değeri (sabit)
    float initial_theta; // Başlangıç açısı (sabit)

    bool initial_pose_set = false; // İlk konumun kaydedildiğini takip etmek için
    bool is_path_complated = true;

    float goal_x;
    float goal_y;
    double epsilon_ = 0.1;

    float path_slope;

    // rclcpp::TimerBase::SharedPtr start_timer;
    rclcpp::TimerBase::SharedPtr turtle_vel_timer;
    rclcpp::TimerBase::SharedPtr new_point_timer;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
    // rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_background_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_vel_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}