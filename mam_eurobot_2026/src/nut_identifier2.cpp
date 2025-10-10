#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <nut_identifier_msgs/msg/nut_position.hpp>
#include <nut_identifier_msgs/msg/nut_position_array.hpp>

class NutIdentifier2 : public rclcpp::Node
{
public:
    NutIdentifier2()
    : Node("nut_identifier2")
    {
        // Déclaration des paramètres avec des valeurs par défaut
        this->declare_parameter<double>("nut_real_diameter_mm", 17.0);
        this->declare_parameter<double>("camera_focal_px", 800.0);
        this->declare_parameter<int>("hsv_hue_min", 100);
        this->declare_parameter<int>("hsv_hue_max", 140);
        this->declare_parameter<int>("hsv_sat_min", 150);
        this->declare_parameter<int>("hsv_sat_max", 255);
        this->declare_parameter<int>("hsv_val_min", 50);
        this->declare_parameter<int>("hsv_val_max", 255);
        this->declare_parameter<double>("min_contour_area", 100.0);

        // Lecture des paramètres au démarrage
        nut_real_diameter_mm_ = this->get_parameter("nut_real_diameter_mm").as_double();
        camera_focal_px_ = this->get_parameter("camera_focal_px").as_double();
        lower_blue_ = cv::Scalar(this->get_parameter("hsv_hue_min").as_int(), this->get_parameter("hsv_sat_min").as_int(), this->get_parameter("hsv_val_min").as_int());
        upper_blue_ = cv::Scalar(this->get_parameter("hsv_hue_max").as_int(), this->get_parameter("hsv_sat_max").as_int(), this->get_parameter("hsv_val_max").as_int());
        min_contour_area_ = this->get_parameter("min_contour_area").as_double();

        RCLCPP_INFO(this->get_logger(), "NutIdentifier2 node started.");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&NutIdentifier2::image_callback, this, std::placeholders::_1));
        nut_pub_ = this->create_publisher<nut_identifier_msgs::msg::NutPositionArray>("/nuts_detected", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat color_mask;
        cv::inRange(hsv, lower_blue_, upper_blue_, color_mask);
 
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        nut_identifier_msgs::msg::NutPositionArray nut_array_msg;

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < min_contour_area_) continue; // Filtre le bruit

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours[i], center, radius);
            float nut_diameter_px = 2.0f * radius;

            // Estimation distance
            float distance_mm = (nut_real_diameter_mm_ * camera_focal_px_) / nut_diameter_px;
            float distance_m = distance_mm / 1000.0;

            // Angle horizontal (centre du nut par rapport au centre image)
            float angle_rad = std::atan2(center.x - frame.cols / 2.0f, camera_focal_px_);

            nut_identifier_msgs::msg::NutPosition nut_msg;
            nut_msg.distance = distance_m;
            nut_msg.angle = angle_rad;

            nut_array_msg.nuts.push_back(nut_msg);
            
            RCLCPP_INFO(this->get_logger(),
                "Nut %zu: diameter_px=%.1f, distance=%.2f m, angle=%.2f rad",
                i, nut_diameter_px, distance_m, angle_rad);
        }

        nut_pub_->publish(nut_array_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<nut_identifier_msgs::msg::NutPositionArray>::SharedPtr nut_pub_;

    // Paramètres
    double nut_real_diameter_mm_;
    double camera_focal_px_;
    cv::Scalar lower_blue_;
    cv::Scalar upper_blue_;
    double min_contour_area_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NutIdentifier2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}