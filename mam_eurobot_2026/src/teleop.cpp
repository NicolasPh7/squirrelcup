#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <algorithm>

class Teleop : public rclcpp::Node
{
public:
    Teleop()
    : Node("teleop")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        joint1_pub_ = this->create_publisher<std_msgs::msg::Float64>("/joint_1/position_cmd", 10);
        joint2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/joint_2/position_cmd", 10);
        joint3_pub_ = this->create_publisher<std_msgs::msg::Float64>("/joint_3/position_cmd", 10);
        joint4_pub_ = this->create_publisher<std_msgs::msg::Float64>("/joint_4/position_cmd", 10);
        joint5_pub_ = this->create_publisher<std_msgs::msg::Float64>("/joint_5/position_cmd", 10);

        arm_1_pub_ = this->create_publisher<std_msgs::msg::Float64>("/arm_1_joint/position_cmd", 10);
        arm_2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/arm_2_joint/position_cmd", 10);

        configureTerminal();
        std::cout << R"(
Control Your Robot!
---------------------------
Moving around:
        w       u
    a   s  d    j
        x
    h       l

w/x : increase/decrease forward velocity
h/l : strafe left/right (mecanum lateral)
a/d : increase/decrease angular velocity
space or s : stop
q/z : linear speed +/-
e/c : angular speed +/-
u/j : joint 1 up/down
i/k : joint 2 up/down
o/l : joint 3 up/down
p/; : joint 4 up/down
r/f : joint 5 up/down
v/b : gripper open/close
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
    double target_linear_ = 0.0, target_lateral_ = 0.0, target_angular_ = 0.0;
    double control_linear_ = 0.0, control_lateral_ = 0.0, control_angular_ = 0.0;
    const double LIN_STEP = 0.05, LAT_STEP = 0.05, ANG_STEP = 0.1;
    const double MAX_LIN = 5.0, MAX_LAT = 5.0, MAX_ANG = 2.84;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint1_pub_, joint2_pub_, joint3_pub_, joint4_pub_, joint5_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_1_pub_, arm_2_pub_;
    const double JOINT_STEP = 0.1; 
    const double JOINT_MIN = -3.1415; 
    const double JOINT_MAX = 3.1415;

    double target_joint1_pos_ = 0.1, target_joint2_pos_ = 0.1, 
           target_joint3_pos_ = 0.1, target_joint4_pos_ = 0.1,
           target_joint5_pos_ = 0.1,
           target_arm_1_pos_ = 0.1, target_arm_2_pos_ = 0.1;

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
            double prev_lateral = target_lateral_;
            double prev_angular = target_angular_;
            double prev_joint1_pos = target_joint1_pos_;
            double prev_joint2_pos = target_joint2_pos_;
            double prev_joint3_pos = target_joint3_pos_;
            double prev_joint4_pos = target_joint4_pos_;
            double prev_joint5_pos = target_joint5_pos_;
            double prev_arm_1_pos = target_arm_1_pos_;

            switch (c) {
                case 'w': target_linear_ += LIN_STEP; break;
                case 'x': target_linear_ -= LIN_STEP; break;
                case 'h': target_lateral_ += LAT_STEP; break;   // strafe left
                case 'l': target_lateral_ -= LAT_STEP; break;   // strafe right
                case 'a': target_angular_ += ANG_STEP; break;
                case 'd': target_angular_ -= ANG_STEP; break;
                case 's':
                case ' ': target_linear_ = 0.0; target_lateral_ = 0.0; target_angular_ = 0.0; break;
                case 'q': target_linear_ += 0.05; break;
                case 'z': target_linear_ -= 0.05; break;
                case 'e': target_angular_ += 0.05; break;
                case 'c': target_angular_ -= 0.05; break;
                case 'u': target_joint1_pos_ += JOINT_STEP; break;
                case 'j': target_joint1_pos_ -= JOINT_STEP; break;
                case 'i': target_joint2_pos_ += JOINT_STEP; break;
                case 'k': target_joint2_pos_ -= JOINT_STEP; break;
                case 'o': target_joint3_pos_ += JOINT_STEP; break;
                case 'm': target_joint3_pos_ -= JOINT_STEP; break;
                case 'p': target_joint4_pos_ += JOINT_STEP; break;
                case ';': target_joint4_pos_ -= JOINT_STEP; break;
                case 'r': target_joint5_pos_ += JOINT_STEP; break;
                case 'f': target_joint5_pos_ -= JOINT_STEP; break;
                case 'v': target_arm_1_pos_ += JOINT_STEP; 
                          target_arm_2_pos_ += JOINT_STEP; break;
                case 'b': target_arm_1_pos_ -= JOINT_STEP; 
                          target_arm_2_pos_ -= JOINT_STEP; break;
                default: continue;
            }

            target_linear_  = std::clamp(target_linear_,  -MAX_LIN, MAX_LIN);
            target_lateral_ = std::clamp(target_lateral_, -MAX_LAT, MAX_LAT);
            target_angular_ = std::clamp(target_angular_, -MAX_ANG, MAX_ANG);
            target_joint1_pos_ = std::clamp(target_joint1_pos_, JOINT_MIN, JOINT_MAX);
            target_joint2_pos_ = std::clamp(target_joint2_pos_, JOINT_MIN, JOINT_MAX);
            target_joint3_pos_ = std::clamp(target_joint3_pos_, JOINT_MIN, JOINT_MAX);
            target_joint4_pos_ = std::clamp(target_joint4_pos_, JOINT_MIN, JOINT_MAX);
            target_joint5_pos_ = std::clamp(target_joint5_pos_, JOINT_MIN, JOINT_MAX);
            target_arm_1_pos_  = std::clamp(target_arm_1_pos_, JOINT_MIN, JOINT_MAX);
            target_arm_2_pos_  = std::clamp(target_arm_2_pos_, JOINT_MIN, JOINT_MAX);

            if (target_linear_ != prev_linear || target_lateral_ != prev_lateral || target_angular_ != prev_angular) {
                RCLCPP_INFO(this->get_logger(),
                    "Target velocity updated → linear: %.2f m/s, lateral: %.2f m/s, angular: %.2f rad/s",
                    target_linear_, target_lateral_, target_angular_);
            }
            if (target_joint1_pos_ != prev_joint1_pos || target_joint2_pos_ != prev_joint2_pos ||
                target_joint3_pos_ != prev_joint3_pos || target_joint4_pos_ != prev_joint4_pos ||
                target_joint5_pos_ != prev_joint5_pos) {
                RCLCPP_INFO(this->get_logger(),
                    "Target Joint position updated → joint1: %.2f, joint2: %.2f, joint3: %.2f, joint4: %.2f, joint5: %.2f",
                    target_joint1_pos_, target_joint2_pos_, target_joint3_pos_, target_joint4_pos_, target_joint5_pos_);
            }
            if (target_arm_1_pos_ != prev_arm_1_pos) {
                RCLCPP_INFO(this->get_logger(),
                    "Target gripper position updated → arm1: %.2f, arm2: %.2f",
                                        target_arm_1_pos_, target_arm_2_pos_);
            }

            // Smooth control update
            control_linear_  += (target_linear_  - control_linear_)  * 0.5;
            control_lateral_ += (target_lateral_ - control_lateral_) * 0.5;
            control_angular_ += (target_angular_ - control_angular_) * 0.5;

            // Publish Twist
            geometry_msgs::msg::Twist twist;
            twist.linear.x = control_linear_;
            twist.linear.y = control_lateral_;   // lateral movement for mecanum
            twist.angular.z = control_angular_;
            pub_->publish(twist);

            // Publish joint positions
            std_msgs::msg::Float64 msg;
            msg.data = target_joint1_pos_; joint1_pub_->publish(msg);
            msg.data = target_joint2_pos_; joint2_pub_->publish(msg);
            msg.data = target_joint3_pos_; joint3_pub_->publish(msg);
            msg.data = target_joint4_pos_; joint4_pub_->publish(msg);
            msg.data = target_joint5_pos_; joint5_pub_->publish(msg);
            msg.data = target_arm_1_pos_; arm_1_pub_->publish(msg);
            msg.data = target_arm_2_pos_; arm_2_pub_->publish(msg);
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

                    