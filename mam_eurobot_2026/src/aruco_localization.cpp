#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

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
            // cv::Mat R_marker_to_cam;
            // cv::Rodrigues(rvecs[i], R_marker_to_cam);
            // cv::Mat t_marker_to_cam = cv::Mat(tvecs[i]);

            // // Invertiere die Transformation: Kamera zur Marker
            // cv::Mat R_cam_to_marker = R_marker_to_cam.t();  // Transponierte Rotation
            // cv::Mat t_cam_to_marker = -R_cam_to_marker * t_marker_to_cam;

            // // Markerposition im Weltkoordinatensystem
            // cv::Vec3d marker_world = marker_map_[id];
            // cv::Mat marker_world_mat = (cv::Mat_<double>(3,1) << marker_world[0], marker_world[1], marker_world[2]);

            // // Kamera (Roboter) Position im Weltkoordinatensystem
            // cv::Mat robot_world_mat = marker_world_mat + R_cam_to_marker * t_marker_to_cam;
            // cv::Vec3d robot_world(robot_world_mat.at<double>(0), robot_world_mat.at<double>(1), robot_world_mat.at<double>(2));
            // geometry_msgs::msg::Pose pose;
            // pose.position.x = robot_world[0];
            // pose.position.y = robot_world[1];
            // pose.position.z = robot_world[2];

            // publishPose(pose);
          }
        }
      }

      if (is_bird_eye_ && !observed_world_positions.empty()) {
        // Berechne mittleren Fehlervektor
        cv::Vec3d total_error(0, 0, 0);
        for (size_t i = 0; i < observed_world_positions.size(); ++i) {
          total_error += expected_world_positions[i] - observed_world_positions[i];
        }
        cv::Vec3d mean_error = total_error * (1.0 / observed_world_positions.size());

        RCLCPP_INFO(this->get_logger(), "Position Mean error correction: [%.4f, %.4f, %.4f]",
            mean_error[0], mean_error[1], mean_error[2]);

        // Finde Marker 0 und korrigiere seine Position
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == 0) {
            geometry_msgs::msg::Pose raw_pose = transformToWorld(tvecs[i], rvecs[i]);
            raw_pose.position.x += mean_error[0];
            raw_pose.position.y += mean_error[1];
            raw_pose.position.z += mean_error[2];
            publishPose(raw_pose);
            break;
          }
        }
      } else if (is_bird_eye_){
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == 0) {
            auto marker_world = transformToWorld(tvecs[i], rvecs[i]); 
            publishPose(marker_world);
            break;
          }
        }
      }

    }

    if (is_bird_eye_ && !robot_visible) {
      publishPose(robot_pose_);
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




  void publishPose(const geometry_msgs::msg::Pose &position) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose = position;
    pose_pub_->publish(pose);

    RCLCPP_INFO(this->get_logger(),
    "Published Pose x=%.3f, y=%.3f, z=%.3f | orientation x=%.3f, y=%.3f, z=%.3f, w=%.3f",
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;

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
