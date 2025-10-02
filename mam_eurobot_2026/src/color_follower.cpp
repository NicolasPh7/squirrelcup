#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorFollower : public rclcpp::Node {
public:
    ColorFollower() : Node("color_follower") {
        RCLCPP_INFO(this->get_logger(), "ColorFollower node started.");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&ColorFollower::image_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received image frame.");

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Blue filter
        cv::Scalar lower_blue(100, 150, 50);
        cv::Scalar upper_blue(140, 255, 255);
        cv::inRange(hsv, lower_blue, upper_blue, mask);

        cv::Moments m = cv::moments(mask, true);
        geometry_msgs::msg::Twist cmd;

        if (m.m00 > 0) {
            int cx = m.m10 / m.m00;
            int width = frame.cols;
            int error = cx - width / 2;

            cmd.linear.x = 0.2;
            cmd.angular.z = -error / 100.0;

            RCLCPP_INFO(this->get_logger(), "Target detected at x=%d (error=%d). Sending cmd_vel: linear=%.2f angular=%.2f",
                        cx, error, cmd.linear.x, cmd.angular.z);
        } else {
            RCLCPP_WARN(this->get_logger(), "No target detected. Robot will stop.");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
