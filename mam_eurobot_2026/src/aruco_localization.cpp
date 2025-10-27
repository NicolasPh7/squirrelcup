#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <unordered_map>
#include <string>

class ArucoLocalization : public rclcpp::Node {
public:
  ArucoLocalization(const std::string &input_topic,
                    const std::string &pose_topic,
                    const std::string &debug_topic,
                    const cv::Vec3d &camera_position,
                    const cv::Vec3d &camera_rpy, bool is_bird_eye)
    : Node("aruco_localization"),
      camera_position_(camera_position),
      camera_bird_eye_rpy_(camera_rpy),
      is_bird_eye_(is_bird_eye)
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic, 10,
      std::bind(&ArucoLocalization::imageCallback, this, std::placeholders::_1));

    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic, 10);
    pose_with_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/nuts", 10);

    marker_map_[20] = cv::Vec3d(0.6, 1.4, 0.0036);
    marker_map_[21] = cv::Vec3d(2.4, 1.4, 0.0036);
    marker_map_[22] = cv::Vec3d(0.6, 0.6, 0.0036);
    marker_map_[23] = cv::Vec3d(2.4, 0.6, 0.0036);

    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    if (is_bird_eye_) {
    camera_position_ = cv::Vec3d(1.5, 0.0, 0.9);
    camera_bird_eye_rpy_ = cv::Vec3d(0.0, 1.07, 1.57);

    camera_matrix_ = (cv::Mat_<double>(3,3) <<
    554.256, 0.0, 960.0,
    0.0,    554.256, 540.0,
    0.0,    0.0,     1.0);
    // camera_matrix_ = (cv::Mat_<double>(3,3) <<
    // 554.256, 0.0, 960.0,
    // 0.0,    554.256, 540.0,
    // 0.0,    0.0,     1.0);
    
  } else {
    camera_matrix_ = (cv::Mat_<double>(3,3) <<
    960.0, 0.0, 960.0,
    0.0,  960.0, 540.0,
    0.0,  0.0,   1.0);
    
    
    // Dynamisch über Odometrie oder tf2
    odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/odometry", 10,
      std::bind(&ArucoLocalization::odomCallback, this, std::placeholders::_1));
  }
  }

private:
  void odomCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    camera_position_ = cv::Vec3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z
    );

    // // Optional: RPY aus Quaternion extrahieren
    // tf2::Quaternion q(
    //   msg->pose.orientation.x,
    //   msg->pose.orientation.y,
    //   msg->pose.orientation.z,
    //   msg->pose.orientation.w);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    robot_pose_ = msg->pose;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    auto parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    cv::Mat undistorted;
    cv::undistort(image, undistorted, camera_matrix_, dist_coeffs_);
    cv::aruco::detectMarkers(undistorted, dictionary, corners, ids, parameters, rejectedCandidates);

    cv::Mat debug_image = undistorted.clone();

    bool robot_visible = false;

    if (!ids.empty()) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      cv::aruco::drawDetectedMarkers(debug_image, corners, ids);

      std::vector<cv::Vec3d> observed_world_positions;
      std::vector<cv::Vec3d> expected_world_positions;

      visualization_msgs::msg::MarkerArray marker_array;
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.frame_id = "map";
      clear_marker.header.stamp = this->now();
      clear_marker.ns = "nuts";
      clear_marker.id = 0;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear_marker);

      marker_pub_->publish(marker_array);
      marker_array.markers.clear();

      for (size_t i = 0; i < ids.size(); ++i) {
        cv::aruco::drawAxis(debug_image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.05);

        int id = ids[i];
        RCLCPP_INFO(this->get_logger(), "Scanned ArUco Marker %d", id);

        if (id == 0) {
          robot_visible = true;
        } else if (marker_map_.count(id)) {
          if (is_bird_eye_) {
            // Berechne Markerposition im Weltkoordinatensystem
            geometry_msgs::msg::Pose estimated_pose = transformToWorld(tvecs[i], rvecs[i]);
            cv::Vec3d estimated_pos(estimated_pose.position.x, estimated_pose.position.y, estimated_pose.position.z);
            observed_world_positions.push_back(estimated_pos);
            expected_world_positions.push_back(marker_map_[id]);

          } else {
            // TODO: tag calibration by the robot
          }
        }
      }

      if (is_bird_eye_) {
        cv::Vec3d mean_variance(0.005, 0.005, 0.005);
        cv::Vec3d mean_error(0.01, 0.01, 0.01);
        
        if (!observed_world_positions.empty()) {
          cv::Vec3d variance(0, 0, 0);
          cv::Vec3d total_error(0, 0, 0);
          // Covariance 
          for (size_t i = 0; i < observed_world_positions.size(); ++i) {
            total_error += expected_world_positions[i] - observed_world_positions[i];

            cv::Vec3d error = expected_world_positions[i] - observed_world_positions[i];
            variance += error.mul(error);
          }
          
          mean_error = total_error * (1.0 / observed_world_positions.size());
          mean_variance = variance * (1.0 / observed_world_positions.size());

          RCLCPP_INFO(this->get_logger(), "Position Mean error correction: [%.4f, %.4f, %.4f]",
              mean_error[0], mean_error[1], mean_error[2]);
          RCLCPP_INFO(this->get_logger(), "Position Covariance: [x=%.4f, y=%.4f, z=%.4f]",
              mean_variance[0], mean_variance[1], mean_variance[2]);
        } 

        int marker_id = 0;
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == 0) {
            geometry_msgs::msg::Pose raw_pose = transformToWorld(tvecs[i], rvecs[i]);
            publishPoseWithCovariance(raw_pose, mean_variance);
          } else {
            geometry_msgs::msg::Pose raw_nut_pose = transformToWorld(tvecs[i], rvecs[i]);
            raw_nut_pose.position.x += mean_error[0];
            raw_nut_pose.position.y += mean_error[1];
            raw_nut_pose.position.z += mean_error[2];
            prepareNuts(marker_array, raw_nut_pose, ids[i], ++marker_id);
          }
        }
        
      }

      marker_pub_->publish(marker_array);
    }

    if (is_bird_eye_ && !robot_visible) {
      cv::Vec3d assumed_cov(0.0100, 0.0100, 0.005);
      publishPoseWithCovariance(robot_pose_, assumed_cov);
    }

    if (!rejectedCandidates.empty()) {
      cv::aruco::drawDetectedMarkers(debug_image, rejectedCandidates, cv::noArray(), cv::Scalar(100, 0, 255));
    }

    auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
    debug_image_pub_->publish(*debug_msg);

  }

  geometry_msgs::msg::Pose transformToWorld(const cv::Vec3d &tvec, const cv::Vec3d &rvec) {
      // Kamera-RPY → Rotation
      double roll = camera_bird_eye_rpy_[0], pitch = camera_bird_eye_rpy_[1], yaw = camera_bird_eye_rpy_[2];
      cv::Mat Rx = (cv::Mat_<double>(3,3) <<
          1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll));
      cv::Mat Ry = (cv::Mat_<double>(3,3) <<
          cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch));
      cv::Mat Rz = (cv::Mat_<double>(3,3) <<
          cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1);

      cv::Mat R_cam = Rz * Ry * Rx;

      // Markerposition relativ zur Kamera
      cv::Mat tvec_cam = cv::Mat(tvec).reshape(1, 3);
      cv::Mat cam_pos = cv::Mat(camera_position_).reshape(1, 3);
      cv::Mat marker_world = R_cam * tvec_cam + cam_pos;

      // Markerrotation relativ zur Kamera
      cv::Mat R_marker_cam;
      cv::Rodrigues(rvec, R_marker_cam);  // rvec → Rotation matrix

      // Markerrotation in Weltkoordinaten
      cv::Mat R_marker_world = R_cam * R_marker_cam;

      // Konvertiere Rotation in Quaternion
      tf2::Matrix3x3 tf_rot(
          R_marker_world.at<double>(0,0), R_marker_world.at<double>(0,1), R_marker_world.at<double>(0,2),
          R_marker_world.at<double>(1,0), R_marker_world.at<double>(1,1), R_marker_world.at<double>(1,2),
          R_marker_world.at<double>(2,0), R_marker_world.at<double>(2,1), R_marker_world.at<double>(2,2)
      );
      tf2::Quaternion q;
      tf_rot.getRotation(q);

      // Rückgabe als Pose
      geometry_msgs::msg::Pose pose;
      pose.position.x = marker_world.at<double>(0);
      pose.position.y = marker_world.at<double>(1);
      pose.position.z = marker_world.at<double>(2);
      pose.orientation = tf2::toMsg(q);
      return pose;
  }

  void publishPoseWithCovariance(const geometry_msgs::msg::Pose &position, const cv::Vec3d &cov_diag) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.pose = position;

    pose.pose.covariance[0] = cov_diag[0]; // x-x
    pose.pose.covariance[7] = cov_diag[1]; // y-y
    pose.pose.covariance[14] = cov_diag[2]; // z-z
    pose.pose.covariance[21] = 0.01; // roll
    pose.pose.covariance[28] = 0.01; // pitch
    pose.pose.covariance[35] = 0.01; // yaw

    pose_with_cov_pub_->publish(pose);

    RCLCPP_INFO(this->get_logger(),
        "Published PoseWithCovariance x=%.3f, y=%.3f, z=%.3f | cov=[%.4f, %.4f, %.4f]",
        pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z,
        cov_diag[0], cov_diag[1], cov_diag[2]);
  }

  void prepareNuts(visualization_msgs::msg::MarkerArray& marker_array, const geometry_msgs::msg::Pose &position, int tag_id, int index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "nuts";
    marker.id = index;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = position.position.x;
    marker.pose.position.y = position.position.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation = position.orientation;

    marker.scale.x = 0.150;
    marker.scale.y = 0.05;
    marker.scale.z = 0.03;

    int r, g, b;
    switch (tag_id) {
      case 36: 
        r = 0.0; g = 0.0; b = 1.0;
        break;
      case 47:
        r = 1.0; g = 1.0; b = 0.0;
        break;
      case 41:
        r = 1.0; g = 1.0; b = 1.0;
        break;
      default:
        r = 0.5; g = 0.5; b = 0.5;
    }

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.8;

    marker_array.markers.push_back(marker);

    RCLCPP_INFO(this->get_logger(),
    "Published Marker x=%.3f, y=%.3f, z=%.3f | orientation x=%.3f, y=%.3f, z=%.3f, w=%.3f",
    position.position.x, position.position.y, position.position.z,
    position.orientation.x, position.orientation.y,
    position.orientation.z, position.orientation.w);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  geometry_msgs::msg::Pose robot_pose_;
  std::unordered_map<int, cv::Vec3d> marker_map_;
  cv::Mat camera_matrix_, dist_coeffs_;
  cv::Vec3d camera_position_, camera_bird_eye_rpy_;

  bool is_bird_eye_;


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto robot_node = std::make_shared<ArucoLocalization>(
    "/camera", "/corrected_pose", "/robot_debug",
    cv::Vec3d(0.0, 0.0, 0.2), cv::Vec3d(0.0, 0.0, 0.0), false);

  auto bird_eye_node = std::make_shared<ArucoLocalization>(
    "/bird_eye", "/corrected_pose", "/bird_eye_debug",
    cv::Vec3d(1.5, 0.0, 0.8), cv::Vec3d(0.0, 1.07, 1.57), true);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_node);
  executor.add_node(bird_eye_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
