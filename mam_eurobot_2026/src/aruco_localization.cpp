#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
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
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>((is_bird_eye_ ? "/nuts" : "/nuts_robot"), 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    marker_map_[20] = cv::Vec3d(0.6, 1.4, 0.0036);
    marker_map_[21] = cv::Vec3d(2.4, 1.4, 0.0036);
    marker_map_[22] = cv::Vec3d(0.6, 0.6, 0.0036);
    marker_map_[23] = cv::Vec3d(2.4, 0.6, 0.0036);

    marker_sizes_ = {
      {0, 0.1},     // Referenzmarker
      {20, 0.1}, {21, 0.1}, {22, 0.1}, {23, 0.1}, // Weltmarker
      {36, 0.04}, {47, 0.04}, {41, 0.04}, // Kistenmarker
      {91, 0.06}, {92, 0.06}, {93, 0.06} //Fix Balises
    };


    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    if (is_bird_eye_) {
    pose_with_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10);
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
    pose_with_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic, 10,
      std::bind(&ArucoLocalization::PoseWithCovarianceStampedCallback, this, std::placeholders::_1));

    camera_matrix_ = (cv::Mat_<double>(3,3) <<
    960.0, 0.0, 960.0,
    0.0,  960.0, 540.0,
    0.0,  0.0,   1.0);
    
  }
  }

private:

  void PoseWithCovarianceStampedCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
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
      std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());

      for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];
        float markerLength = marker_sizes_.count(id) ? marker_sizes_[id] : 0.1; // Default fallback
        std::vector<std::vector<cv::Point2f>> single_corner = { corners[i] };
        std::vector<cv::Vec3d> rvec_single, tvec_single;

        cv::aruco::estimatePoseSingleMarkers(single_corner, markerLength, camera_matrix_, dist_coeffs_, rvec_single, tvec_single);
        rvecs[i] = rvec_single[0];
        tvecs[i] = tvec_single[0];
      }

      cv::aruco::drawDetectedMarkers(debug_image, corners, ids);

      visualization_msgs::msg::MarkerArray marker_array;
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.frame_id = "map";
      clear_marker.header.stamp = msg->header.stamp;
      clear_marker.ns = "nuts";
      clear_marker.id = 0;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear_marker);

      marker_pub_->publish(marker_array);
      marker_array.markers.clear();

      std::vector<cv::Point3f> objectPoints;
      std::vector<cv::Point2f> imagePoints;

      for (size_t i = 0; i < ids.size(); ++i) {
        cv::aruco::drawAxis(debug_image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.05);

        int id = ids[i];
        RCLCPP_INFO(this->get_logger(), "Scanned ArUco Marker %d", id);

        if (id == 0) {
          robot_visible = true;
        } else if (marker_map_.count(id)) {
          if (is_bird_eye_) {
            int id = ids[i];
            if (marker_map_.count(id)) {
              cv::Point2f center(0, 0);
              for (const auto& pt : corners[i]) center += pt;
              center *= 0.25;

              imagePoints.push_back(center);
              objectPoints.push_back(cv::Point3f(marker_map_[id]));
            }
          } else {
            // TODO: tag calibration by the robot
          }
        }
      }

      int marker_id = 0;

      if (is_bird_eye_) {
        double mean_variance;
        double mean_error;

        // solvePnP zur Kamerapose-Schätzung
        if (objectPoints.size() >= 4) {
          cv::Vec3d rvec, tvec;
          cv::solvePnP(objectPoints, imagePoints, camera_matrix_, dist_coeffs_, rvec, tvec);

          std::vector<cv::Point2f> projectedPoints;
          cv::projectPoints(objectPoints, rvec, tvec, camera_matrix_, dist_coeffs_, projectedPoints);

          double totalError = 0;
          double variance = 0;
          for (size_t i = 0; i < imagePoints.size(); ++i) {
            auto error = cv::norm(imagePoints[i] - projectedPoints[i]);
            totalError += error;
            variance += error*error;
          }
          
          mean_error = totalError / imagePoints.size();
          mean_variance = variance / imagePoints.size();

          RCLCPP_INFO(this->get_logger(), "Position Mean error correction: %.4f pixels", mean_error);
          RCLCPP_INFO(this->get_logger(), "Position Covariance: %.4f pixels square",mean_variance);

          geometry_msgs::msg::Pose camera_pose = cameraPoseToWorld(tvec, rvec);

          // publishPoseWithCovariance(camera_pose, mean_variance);

          for (size_t i = 0; i < ids.size(); ++i) {
            int id = static_cast<int>(ids[i]);
            if (id == 0) {
              geometry_msgs::msg::Pose raw_pose = transformTagToWorld(tvecs[i], rvecs[i], invertPose(camera_pose));
              publishPoseWithCovariance(raw_pose, mean_variance);
            } else if (!(marker_map_.count(id))) {
              geometry_msgs::msg::Pose raw_nut_pose = transformTagToWorld(tvecs[i], rvecs[i], invertPose(camera_pose));
              prepareNuts(marker_array, raw_nut_pose, id, ++marker_id);
            }
          }
        }

      } else { // Robot view
          for (size_t i = 0; i < ids.size(); ++i) {
            int id = static_cast<int>(ids[i]);
            if (!(marker_map_.count(id))) {
              geometry_msgs::msg::Pose raw_nut_pose = transformTagToWorld(tvecs[i], rvecs[i], invertPose(robot_pose_));
              RCLCPP_INFO(this->get_logger(), "Robot sees nuts");
              prepareNuts(marker_array, raw_nut_pose, id, ++marker_id);
            }
          }
      }

      marker_pub_->publish(marker_array);
    }

    if (is_bird_eye_ && !robot_visible) {
      RCLCPP_INFO(this->get_logger(), "Robot not visible. Falling back to odometry");
      auto assumed_cov = 0.0100;
      publishPoseWithCovariance(robot_pose_, assumed_cov);
    }

    if (!rejectedCandidates.empty()) {
      cv::aruco::drawDetectedMarkers(debug_image, rejectedCandidates, cv::noArray(), cv::Scalar(100, 0, 255));
    }

    auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
    debug_image_pub_->publish(*debug_msg);

  }

  geometry_msgs::msg::Pose invertPose(const geometry_msgs::msg::Pose& pose) {
      tf2::Transform tf_pose;
      tf2::fromMsg(pose, tf_pose);

      tf2::Transform tf_inv = tf_pose.inverse();

      geometry_msgs::msg::Pose inverted_pose;
      inverted_pose.position.x = tf_inv.getOrigin().x();
      inverted_pose.position.y = tf_inv.getOrigin().y();
      inverted_pose.position.z = tf_inv.getOrigin().z();

      tf2::Quaternion q = tf_inv.getRotation();
      inverted_pose.orientation = tf2::toMsg(q);

      return inverted_pose;
  }


  geometry_msgs::msg::Pose cameraPoseToWorld(const cv::Vec3d &tvec, const cv::Vec3d &rvec) {
    // Rotation
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    tf2::Matrix3x3 tf_rot(
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    );
    tf2::Quaternion q;
    tf_rot.getRotation(q);

    geometry_msgs::msg::Pose pose;
    pose.position.x = tvec[0];
    pose.position.y = tvec[1];
    pose.position.z = tvec[2];
    pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(),
        "Computed Camera Pose x=%.3f, y=%.3f, z=%.3f",
       pose.position.x, pose.position.y, pose.position.z);

    return pose;
  }


  geometry_msgs::msg::Pose transformTagToWorld(const cv::Vec3d &tag_tvec_cam, const cv::Vec3d &tag_rvec_cam,
                                             const geometry_msgs::msg::Pose &camera_pose_world) {
    // Kamera-Rotation als Matrix
    tf2::Quaternion q_cam;
    tf2::fromMsg(camera_pose_world.orientation, q_cam);
    tf2::Matrix3x3 tf_R_cam(q_cam);

    cv::Mat R_cam = (cv::Mat_<double>(3,3) <<
        tf_R_cam[0][0], tf_R_cam[0][1], tf_R_cam[0][2],
        tf_R_cam[1][0], tf_R_cam[1][1], tf_R_cam[1][2],
        tf_R_cam[2][0], tf_R_cam[2][1], tf_R_cam[2][2]);

    // Kamera-Position als Vektor
    cv::Mat cam_pos = (cv::Mat_<double>(3,1) <<
        camera_pose_world.position.x,
        camera_pose_world.position.y,
        camera_pose_world.position.z);

    // Markerposition relativ zur Kamera
    cv::Mat tvec_cam = cv::Mat(tag_tvec_cam).reshape(1, 3);

    // Markerposition in Weltkoordinaten
    cv::Mat marker_world = R_cam * tvec_cam + cam_pos;

    // Markerrotation relativ zur Kamera
    cv::Mat R_marker_cam;
    cv::Rodrigues(tag_rvec_cam, R_marker_cam);

    // Markerrotation in Weltkoordinaten
    cv::Mat R_marker_world = R_cam * R_marker_cam;

    // Rotation → Quaternion
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

  void publishPoseWithCovariance(const geometry_msgs::msg::Pose &position, double cov_diag) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
  rclcpp::Time now = this->now();
    pose.header.frame_id = "map";
    pose.header.stamp = now;
    pose.pose.pose = position;

    pose.pose.covariance[0] = cov_diag; // x-x
    pose.pose.covariance[7] = cov_diag; // y-y
    pose.pose.covariance[14] = cov_diag; // z-z
    pose.pose.covariance[21] = 0.01; // roll
    pose.pose.covariance[28] = 0.01; // pitch
    pose.pose.covariance[35] = 0.01; // yaw

    udpateBaseLinkTF(position);
    pose_with_cov_pub_->publish(pose);

    RCLCPP_INFO(this->get_logger(),
        "Published PoseWithCovariance x=%.3f, y=%.3f, z=%.3f | cov=%.4f",
        pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z,
        cov_diag);
  }

  void udpateBaseLinkTF(const geometry_msgs::msg::Pose &pose) {
    geometry_msgs::msg::TransformStamped t;
  rclcpp::Time now = this->now();    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = pose.position.x;
    t.transform.translation.y = pose.position.y;
    t.transform.translation.z = pose.position.z;
    t.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  void prepareNuts(visualization_msgs::msg::MarkerArray& marker_array, const geometry_msgs::msg::Pose &position, int tag_id, int index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
  rclcpp::Time now = this->now();    marker.header.stamp = now;
    marker.ns = "nuts";
    marker.id = index;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = position.position.x;
    marker.pose.position.y = position.position.y;
    marker.pose.position.z = position.position.z;
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
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  
  geometry_msgs::msg::Pose robot_pose_;
  std::unordered_map<int, cv::Vec3d> marker_map_;
  std::unordered_map<int, float> marker_sizes_;
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
