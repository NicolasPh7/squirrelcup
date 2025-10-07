#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>

class Teleop : public rclcpp::Node
{
public:
    Teleop()
    : Node("teleop")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        configureTerminal();
        std::cout << R"(
Control Your Robot!
---------------------------
Moving around:
                w
     a    s    d
                x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space or s : stop
q/z : linear speed +/-
e/c : angular speed +/-
CTRL-C to quit
)" << std::endl;
        teleop_thread_ = std::thread(&Teleop::run, this);
    }

    ~Teleop()
    {
        if (teleop_thread_.joinable()) {
            teleop_thread_.join();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::thread teleop_thread_;
    double target_linear_ = 0.0, target_angular_ = 0.0;
    double control_linear_ = 0.0, control_angular_ = 0.0;
    const double LIN_STEP = 0.01, ANG_STEP = 0.1;
    const double MAX_LIN = 0.26, MAX_ANG = 2.84;

    void configureTerminal()
    {
        termios raw;
        tcgetattr(STDIN_FILENO, &raw);
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    void run()
    {
        char c;
        while (rclcpp::ok()) {
            c = getchar();
            double prev_linear = target_linear_;
            double prev_angular = target_angular_;

            switch (c) {
                case 'w': target_linear_ += LIN_STEP; break;
                case 'x': target_linear_ -= LIN_STEP; break;
                case 'a': target_angular_ += ANG_STEP; break;
                case 'd': target_angular_ -= ANG_STEP; break;
                case 's':
                case ' ': target_linear_ = 0.0; target_angular_ = 0.0; break;
                case 'q': target_linear_ += 0.05; break;
                case 'z': target_linear_ -= 0.05; break;
                case 'e': target_angular_ += 0.05; break;
                case 'c': target_angular_ -= 0.05; break;
                default: continue;
            }

            target_linear_ = std::clamp(target_linear_, -MAX_LIN, MAX_LIN);
            target_angular_ = std::clamp(target_angular_, -MAX_ANG, MAX_ANG);

            if (target_linear_ != prev_linear || target_angular_ != prev_angular) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Target velocity updated → linear: %.2f m/s, angular: %.2f rad/s",
                    target_linear_, target_angular_);
            }

            control_linear_ += (target_linear_ - control_linear_) * 0.5;
            control_angular_ += (target_angular_ - control_angular_) * 0.5;

            geometry_msgs::msg::Twist twist;
            twist.linear.x = control_linear_;
            twist.angular.z = control_angular_;
            pub_->publish(twist);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
