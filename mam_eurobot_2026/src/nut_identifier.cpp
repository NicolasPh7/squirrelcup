#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class Nut_Identifier : public rclcpp::Node
{
public:
    Nut_Identifier()
    : Node("nut_identifier")
    {
        RCLCPP_INFO(this->get_logger(), "Nut_Identifier node started.");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&Nut_Identifier::image_callback, this, std::placeholders::_1));

        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/debug/contours_image", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Nut_Identifier::odom_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/nut_markers", 10);


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

        cv::Mat debug_image = frame.clone();

        cv::Mat gray,laplacian, sharp_edges, edge_mask;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::Laplacian(gray, laplacian, CV_16S, 3);
        cv::convertScaleAbs(laplacian, sharp_edges);
        cv::threshold(sharp_edges, edge_mask, 5, 255, cv::THRESH_BINARY);

        cv::Mat hsv, color_mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar lower_blue(100, 150, 50);
        cv::Scalar upper_blue(140, 255, 255);
        cv::inRange(hsv, lower_blue, upper_blue, color_mask);

        cv::Mat combined_mask;
        cv::bitwise_and(edge_mask, color_mask, combined_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(this->get_logger(), "Found %zu contours.", contours.size());

        double max_area = 100;


        // clearMarkers();
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx, 15.0, true);

            if (area > max_area && approx.size() >= 4 && approx.size() <= 20) { 
                max_area = area;
                
                float distance_m, angle_rad, angle_to_camera_rad;
                bool isNut = estimatePosition(frame, contours[i], distance_m, angle_rad, angle_to_camera_rad);
                
                if (isNut) {
                    cv::drawContours(debug_image, contours, i, cv::Scalar(0, 0, 255), 4);  // Red

                    RCLCPP_INFO(this->get_logger(),
                        "Nut detected: dist=%.2f m, angle=%.2f rad, tilt=%.2f rad",
                    distance_m, angle_rad, angle_to_camera_rad);
    
                    prepareMarker(marker_array, i, distance_m, angle_rad, angle_to_camera_rad, 0.0f, 0.0f, 1.0f);
                }

            }
        }

        std_msgs::msg::Header header;
        header.stamp = msg->header.stamp;
        header.frame_id = msg->header.frame_id;

        sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();

        debug_image_pub_->publish(*debug_msg);

        marker_pub_->publish(marker_array);

    }

    bool estimatePosition(cv::Mat& frame, std::vector<cv::Point>& contours, float& distance, float& angle, float& tilt) {
        cv::Moments m = cv::moments(contours);
        int cx = m.m10 / m.m00;
        //int error = cx - frame.cols / 2;

        // angle to camera
        // formula: focal_py = width / (2 * tan(horizontal_fov * 2))
        float focal_px = 554.0f;
        float angle_rad = std::atan2(cx - frame.cols / 2.0f, focal_px);

        //tilt of nut
        cv::Rect bbox = cv::boundingRect(contours);
        float nut_length_mm = 150.0f;
        float nut_depth_mm = 50.0f;
        float projected_length_px = static_cast<float>(bbox.width);
        float projected_length_mm = (projected_length_px * nut_length_mm) / focal_px;
        float angle_to_camera_rad = std::acos(std::clamp(projected_length_mm / nut_length_mm, -1.0f, 1.0f));
    
        //distance approx
        float visible_mm = nut_length_mm * std::cos(angle_to_camera_rad) + nut_depth_mm * std::sin(angle_to_camera_rad);
        float distance_mm = (visible_mm * focal_px) / projected_length_px;
        float distance_m = distance_mm / 1000.0f;

        distance = distance_m;
        angle = angle_rad;
        tilt = angle_to_camera_rad;

        return projected_length_mm >= 30.0f && projected_length_mm <=180.0f;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_current_pose_ = msg->pose.pose;
        odom_timestamp_ = msg-> header.stamp;
    }

    void prepareMarker(visualization_msgs::msg::MarkerArray& marker_array, int id, float distance, float angle, float tilt, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = odom_timestamp_;
        marker.ns = "nut";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        float dx_local = distance * std::cos(angle);
        float dy_local = distance * std::sin(angle);

        // Roboter-Yaw aus aktueller Pose
        tf2::Quaternion q_robot;
        tf2::fromMsg(robot_current_pose_.orientation, q_robot);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_robot).getRPY(roll, pitch, yaw);

        // Transformation in Weltkoordinaten
        float dx_world = dx_local * std::cos(yaw) - dy_local * std::sin(yaw);
        float dy_world = dx_local * std::sin(yaw) + dy_local * std::cos(yaw);

        marker.pose.position.x = robot_current_pose_.position.x + dx_world;
        marker.pose.position.y = robot_current_pose_.position.y + dy_world;
        marker.pose.position.z = robot_current_pose_.position.z;

        // Orientierung aus Tilt-Winkel
        tf2::Quaternion q;
        q.setRPY(0, 0, tilt + M_PI_2 );  // Roll=0, Pitch=Tilt, Yaw=angle
        marker.pose.orientation = tf2::toMsg(q);

        // Größe und Farbe
        marker.scale.x = 0.15;
        marker.scale.y = 0.05;
        marker.scale.z = 0.03;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        RCLCPP_INFO(this->get_logger(),
            "Marker[%d] @ world: x=%.2f, y=%.2f, z=%.2f | local dx=%.2f, dy=%.2f | yaw=%.2f rad | tilt=%.2f rad | angle=%.2f rad",
            id,
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
            dx_local,
            dy_local,
            yaw,
            tilt,
            angle
        );
        

        marker_array.markers.push_back(marker);
    }

    void clearMarkers() {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.header.frame_id = "map";
        delete_marker.header.stamp = this->now();

        visualization_msgs::msg::MarkerArray clear_array;
        clear_array.markers.push_back(delete_marker);
        marker_pub_->publish(clear_array);       
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    geometry_msgs::msg::Pose robot_current_pose_;
    builtin_interfaces::msg::Time odom_timestamp_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nut_Identifier>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
