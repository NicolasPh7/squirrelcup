#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "trajectory_planner.hpp"  

#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node {
public:
  RobotController() : Node("robot_controller"), state_(State::SEARCHING), current_index_(0) {
    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/supposed_objects", 10, std::bind(&RobotController::markerCallback, this, std::placeholders::_1));

    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "/odom", 10, std::bind(&RobotController::odomCallback, this, std::placeholders::_1));
    
    poseCorrectionSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/corrected_pose", 10, std::bind(&RobotController::correctedPoseCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&RobotController::controlLoop, this));

    initPositions() ;

    RCLCPP_INFO(this->get_logger(), "RobotController node initialized.");
  }

  void setPlanner(std::shared_ptr<TrajectoryPlanner> planner) {
    planner_ = planner;
  }

private:
  void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    planner_->clearMarkers();
    for (const auto& marker : msg->markers) {
      planner_->addMarker(marker);
    }
  }

  void correctedPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    planner_->updateCurrentPosition(current_pose_);
    
  }

  void controlLoop() {
    switch (state_) {
      case State::SEARCHING:
      RCLCPP_INFO(this->get_logger(), "[SEARCHING] Checking for available markers...");
      path_ = planner_->generateNearestMarkerPath(reached_, marker_id_);
      if (planner_->setGoal()) {
        stopRobot();
        state_ = State::APPROACHING;
      } else {
        followPath();
      }
        break;

      case State::APPROACHING:
        RCLCPP_INFO(this->get_logger(), "[APPROACHING] Driving to target...");
        
        if(!planner_->setGoal()) {
          stopRobot();
          state_ = State::SEARCHING;
          break;
        }

        path_ = planner_->planTrajectory(reached_);
        if (reached_) {
          stopRobot();
          wait_time_ = this->now();
          state_ = State::COLLECTING;
        } else {
          followPath();
        }
        break;

      case State::COLLECTING:
        RCLCPP_INFO(this->get_logger(), "[COLLECTING] Waiting...");
        if ((this->now() - wait_time_).seconds() > 3.0) {
          state_ = State::PREPARE_RETURN;
        }
        break;

      case State::PREPARE_RETURN:
        RCLCPP_INFO(this->get_logger(), "[PREPARE_RETURN] Driving to nearest marker...");
        path_ = planner_->generateNearestMarkerPath(reached_, marker_id_);
        if (reached_) {
          stopRobot();
          state_ = State::RETURNING;
        } else {
          followPath();
        }
        break;

      case State::RETURNING:
        RCLCPP_INFO(this->get_logger(), "[RETURNING] Driving to nest...");
          planner_->setHomePosition(planner_->computeTranslationPose(home_position_, current_pose_));
          path_ = planner_->planWayHome();
          if (reachedHome()) {
            stopRobot();
            wait_time_ = this->now();
            state_ = State::PLACING;
          } else {
            followPath();
          }
        break;

      case State::PLACING:
        RCLCPP_INFO(this->get_logger(), "[PLACE] Waiting...");
        if ((this->now() - wait_time_).seconds() > 3.0) {
          state_ = State::PREPARE_SEARCH;
        }
        break;
      
      case State::PREPARE_SEARCH:
        RCLCPP_INFO(this->get_logger(), "[PREPARE_SEARCH] Preparing Search...");
        path_ = planner_->generateNearestMarkerPath(reached_, marker_id_);
        if (reached_) {
          stopRobot();
          state_ = State::SEARCHING;
        } else {
          followPath();
        }
        break;
        
    }
  }

  visualization_msgs::msg::Marker createMarkerFromPose(const geometry_msgs::msg::Pose& pose) {
    visualization_msgs::msg::Marker marker;
    marker.pose = pose;
    return marker;
  }

  void stopRobot() {
    current_index_ = 0;
    reached_ = false;
    marker_id_ = -1;
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    current_index_ = 0;
    cmd_pub_->publish(cmd);
  }


  void followPath() {
    if (current_index_ >= path_.size()) {
      stopRobot();
      return;
    }
  
    const auto& target_pose = path_[current_index_].pose;
    double dx = target_pose.position.x;
    double dy = target_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
  
    // Ziel erreicht?
    if (distance < 0.01) {
      current_index_++;
      return;
    }
  
    // Yaw-Berechnung
    double target_yaw = std::atan2(dy, dx);
    double yaw_error = target_yaw;
  
    // Normalisiere Yaw-Fehler auf [-π, π]
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
  
    // Regelparameter
    const double max_linear_speed = 0.2;
    // const double max_angular_speed = 1.2;
    // const double linear_kp = 0.8;
    // const double angular_kp = 2.0;
    // const double angular_deadband = 0.05;
  
    // Berechne Steuerbefehle
    double linear_speed = max_linear_speed;
    double angular_speed = yaw_error;
  
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_speed;
    cmd.angular.z = angular_speed;
    cmd_pub_->publish(cmd);
  }
  
  geometry_msgs::msg::Pose invertPose(const geometry_msgs::msg::Pose& pose) {
    tf2::Transform tf_pose;
    tf2::fromMsg(pose, tf_pose);

    // Invert the transform
    tf2::Transform tf_inverse = tf_pose.inverse();

    // Convert back to geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose inverse_pose;
    tf2::toMsg(tf_inverse, inverse_pose);
    return inverse_pose;
  }




  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  bool reachedHome(double threshold = 0.4) {
    double dx = current_pose_.position.x - home_position_.position.x;
    double dy = current_pose_.position.y - home_position_.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    return dist <= threshold;
  }

  bool inPreparedPosition(double threshold = 0.05) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -1.57079633);  // Roll, Pitch, Yaw
    return reachedHome() 
      && (std::abs(start_position_.orientation.x) <= std::abs(q.x()) + threshold)
      && (std::abs(start_position_.orientation.y) <= std::abs(q.y()) + threshold)
      && (std::abs(start_position_.orientation.z) <= std::abs(q.z()) + threshold)
      && (std::abs(start_position_.orientation.w) <= std::abs(q.w()) + threshold);
  }

  void initPositions() {
    // Home-Position (nur Translation)
    home_position_.position.x = 2.8;
    home_position_.position.y = 1.9;
    home_position_.position.z = 0.0;
    home_position_.orientation.x = 0.0;
    home_position_.orientation.y = 0.0;
    home_position_.orientation.z = 0.0;
    home_position_.orientation.w = 1.0;

    // Start-Position (Translation + Rotation um Z-Achse)
    start_position_.position = home_position_.position;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -1.57079633);  // Roll, Pitch, Yaw
    start_position_.orientation.x = q.x();
    start_position_.orientation.y = q.y();
    start_position_.orientation.z = q.z();
    start_position_.orientation.w = q.w();
  }

  enum class State { SEARCHING, APPROACHING, COLLECTING, PREPARE_RETURN, RETURNING, PLACING, PREPARE_SEARCH};
  State state_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseCorrectionSub_;


  std::shared_ptr<TrajectoryPlanner> planner_;

  geometry_msgs::msg::Pose home_position_;
  geometry_msgs::msg::Pose start_position_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose current_goal_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_index_;
  rclcpp::Time wait_time_;

  bool reached_ = false;
  int marker_id_ = -1;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  auto planner = std::make_shared<TrajectoryPlanner>(node);
  node -> setPlanner(planner);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}