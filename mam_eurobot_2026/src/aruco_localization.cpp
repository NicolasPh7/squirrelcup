#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <unordered_map>

class ArucoLocalization : public rclcpp::Node {
public:
  ArucoLocalization() : Node("aruco_localization") {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera", 10,
      std::bind(&ArucoLocalization::imageCallback, this, std::placeholders::_1));

    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_debug_image", 10);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/corrected_pose", 10);

    //https://www.eurobot.org/wp-content/uploads/2025/10/Eurobot2026_Rules_1.0_FR.pdf
    marker_map_[20] = cv::Vec3d(0.6, 1.4, 0.0036);
    marker_map_[21] = cv::Vec3d(2.4, 1.4, 0.0036);
    marker_map_[22] = cv::Vec3d(0.6, 0.6, 0.0036);
    marker_map_[23] = cv::Vec3d(2.4, 0.6, 0.0036);

    //TODO: camera calibration
    camera_matrix_ = (cv::Mat_<double>(3,3) << 
            554.3827, 0.0,     320.0,
            0.0,      415.787, 240.0,
            0.0,      0.0,     1.0);

    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    // cv::Mat gray;
    // cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    auto parameters = cv::aruco::DetectorParameters::create();
    parameters->adaptiveThreshWinSizeMin = 3;
    parameters->adaptiveThreshWinSizeMax = 23;
    parameters->adaptiveThreshWinSizeStep = 10;
    parameters->minMarkerPerimeterRate = 0.03;
    parameters->maxMarkerPerimeterRate = 4.0;
    parameters->polygonalApproxAccuracyRate = 0.05;
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    cv::Mat undistorted;
    cv::undistort(image, undistorted, camera_matrix_, dist_coeffs_);
    cv::aruco::detectMarkers(undistorted, dictionary, corners, ids, parameters, rejectedCandidates);

    cv::Mat debug_image = undistorted.clone();

    if (!ids.empty()) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
      for (size_t i = 0; i < ids.size(); ++i) {
        cv::aruco::drawAxis(debug_image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.05);

        RCLCPP_INFO(this->get_logger(), "Scanned ArUco Marker %d", ids[i]);
        int id = ids[i];
        if (marker_map_.count(id)) {
          cv::Vec3d marker_world = marker_map_[id];
          cv::Vec3d robot_world = computeRobotPose(marker_world, tvecs[i], rvecs[i]);
          publishPose(robot_world);
        }
      }
    }

    if (!rejectedCandidates.empty()) {
      cv::aruco::drawDetectedMarkers(debug_image, rejectedCandidates, cv::noArray(), cv::Scalar(100, 0, 255));
    }

    auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
    debug_image_pub_->publish(*debug_msg);
  }


  cv::Vec3d computeRobotPose(const cv::Vec3d &marker_world, const cv::Vec3d &tvec, const cv::Vec3d &rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat rel_mat = (R.t() * cv::Mat(tvec));
    rel_mat = rel_mat.reshape(1, 3); 
    cv::Vec3d rel_translation = rel_mat;
    return marker_world + rel_translation;
  }

  void publishPose(const cv::Vec3d &position) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = position[0];
    pose.pose.position.y = position[1];
    pose.pose.position.z = position[2];
    pose.pose.orientation.w = 1.0;  // Dummy orientation
    pose_pub_->publish(pose);

    RCLCPP_INFO(
      this->get_logger(), "Corrected Position x=%.1f, y=%.1f, w=%.1f",
      position[0], position[1], position[2]);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  std::unordered_map<int, cv::Vec3d> marker_map_;
  cv::Mat camera_matrix_, dist_coeffs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

