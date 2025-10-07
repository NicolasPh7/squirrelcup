#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TestCppNode : public rclcpp::Node
{
public:
    TestCppNode()
    : Node("test_cpp_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from C++!");

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TestCppNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;
        bool flag_ = false;
        if (count_ < 10) {              // Publish Twist for 5 seconds
            msg.linear.x = 0.50;                   // forward velocity
            msg.angular.z = 1.57;                  // angular velocity
            RCLCPP_INFO(this->get_logger(), "Moving in a circle...");
        } else {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Stopping the robot.");
            flag_ = true;
        }
        publisher_->publish(msg);
        if (flag_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down node...");
            rclcpp::shutdown();
            return;
        }
        count_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestCppNode>());
    rclcpp::shutdown();
    return 0;
}