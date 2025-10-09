#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorFollower : public rclcpp::Node
{
public:
    ColorFollower()
    : Node("color_follower")
    {
        RCLCPP_INFO(this->get_logger(), "ColorFollower node started.");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&ColorFollower::image_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image frame.");

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat laplacian, sharp_edges;
        cv::Laplacian(gray, laplacian, CV_16S, 3);
        cv::convertScaleAbs(laplacian, sharp_edges);

        cv::Mat edge_mask;
        cv::threshold(sharp_edges, edge_mask, 25, 255, cv::THRESH_BINARY);
        int edge_pixels = cv::countNonZero(edge_mask);
                RCLCPP_INFO(this->get_logger(), "Sharp edge pixel count: %d", edge_pixels);

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower_blue(100, 150, 50);
        cv::Scalar upper_blue(140, 255, 255);
        cv::Mat color_mask;
        cv::inRange(hsv, lower_blue, upper_blue, color_mask);
        int blue_pixels = cv::countNonZero(color_mask);
                RCLCPP_INFO(this->get_logger(), "Blue mask pixel count: %d", blue_pixels);

        cv::Mat combined_mask;
        cv::bitwise_and(edge_mask, color_mask, combined_mask);


        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(this->get_logger(), "Found %zu contours.", contours.size());

        int best_index = -1;
        double max_area = 0;

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);

            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx, 5.0, true);

            RCLCPP_DEBUG(this->get_logger(),
                "Contour %zu: area=%.2f, approxPoly=%d", i, area, (int)approx.size());

            if (area > max_area && approx.size() >= 2 && approx.size() <= 12) { 
            // if (area > max_area) { 
                max_area = area;
                best_index = i;
                RCLCPP_INFO(this->get_logger(),
                    "Contour %zu selected as best candidate.", i);
            }
        }

        geometry_msgs::msg::Twist cmd;

        if (best_index != -1) {
            cv::Moments m = cv::moments(contours[best_index]);
            int cx = m.m10 / m.m00;
            int error = cx - frame.cols / 2;

            cmd.linear.x = 0.5;
            cmd.angular.z = -error / 100.0;

            RCLCPP_INFO(this->get_logger(),
                "Target detected at x=%d (error=%d). Sending cmd_vel: linear=%.2f angular=%.2f",
                cx, error, cmd.linear.x, cmd.angular.z);
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "No valid target detected. Robot will stop.");
        }

        cmd_pub_->publish(cmd);
    }


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
