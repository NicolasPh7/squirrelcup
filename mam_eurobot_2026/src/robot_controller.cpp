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

#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node {
public:
  RobotController() : Node("robot_controller"), state_(State::SEARCHING), current_index_(0) {
    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/nut_markers", 10, std::bind(&RobotController::markerCallback, this, std::placeholders::_1));

    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/current_goal_marker", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&RobotController::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10, std::bind(&RobotController::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(1s, std::bind(&RobotController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "RobotController node initialized.");
  }

private:
  void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    nuts_.clear();
    for (const auto& marker : msg->markers) {
      nuts_.push_back(marker.pose);
    }
    // RCLCPP_INFO(this->get_logger(), "Received %zu nuts from /nut_markers.", nuts_.size());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    if (start_pose_.position.x == 0.0 && start_pose_.position.y == 0.0) {
      start_pose_ = current_pose_;
    //   RCLCPP_INFO(this->get_logger(), "Start pose initialized at (%.2f, %.2f).",
    //               start_pose_.position.x, start_pose_.position.y);
    }
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = msg->poses;
    current_index_ = 0;
    // RCLCPP_INFO(this->get_logger(), "Received path with %zu poses.", path_.size());
  }

  void publishGoalMarker(const geometry_msgs::msg::Pose& goal_pose) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "goal_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = goal_pose;
    marker.scale.x = 0.05;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    goal_marker_pub_->publish(marker);
    // RCLCPP_INFO(this->get_logger(), "Published goal marker at (%.2f, %.2f).",
    //             goal_pose.position.x, goal_pose.position.y);
  }

  void controlLoop() {
    switch (state_) {
      case State::SEARCHING:
        RCLCPP_INFO(this->get_logger(), "[SEARCHING] Nuts available: %zu", nuts_.size());
        if (!nuts_.empty()) {
          current_goal_pose_ = nuts_.front();
          nuts_.erase(nuts_.begin());
          RCLCPP_INFO(this->get_logger(), "Target acquired at (%.2f, %.2f). Transitioning to APPROACHING.",
                      current_goal_pose_.position.x, current_goal_pose_.position.y);
          publishGoalMarker(current_goal_pose_);
          state_ = State::APPROACHING;
        } else {
          searchForNuts();
        }
        break;

      case State::APPROACHING:
        RCLCPP_INFO(this->get_logger(), "[APPROACHING] Driving to target at (%.2f, %.2f).",
                    current_goal_pose_.position.x, current_goal_pose_.position.y);
        if (reachedTarget(current_goal_pose_)) {
          RCLCPP_INFO(this->get_logger(), "Reached target. Transitioning to COLLECTING.");
          stopRobot();
          collect_start_time_ = this->now();
          state_ = State::COLLECTING;
        } else {
          followPath();
        }
        break;

      case State::COLLECTING:
        RCLCPP_INFO(this->get_logger(), "[COLLECTING] Waiting for 2 seconds...");
        if ((this->now() - collect_start_time_).seconds() > 2.0) {
          current_goal_pose_ = start_pose_;
          RCLCPP_INFO(this->get_logger(), "Collection complete. Returning to nest at (%.2f, %.2f).",
                      start_pose_.position.x, start_pose_.position.y);
          state_ = State::RETURNING;
        }
        break;

      case State::RETURNING:
        RCLCPP_INFO(this->get_logger(), "[RETURNING] Driving to nest.");
        publishGoalMarker(start_pose_);
        if (reachedTarget(start_pose_)) {
          RCLCPP_INFO(this->get_logger(), "Returned to nest. Transitioning to SEARCHING.");
          state_ = State::SEARCHING;
        } else {
          followPath();
        }
        break;
    }
  }

  void searchForNuts() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.2;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Driving forward.");
  }

  void stopRobot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Robot stopped.");
  }

  void driveTo(const geometry_msgs::msg::Pose& target_pose) {
    double dx = target_pose.position.x - current_pose_.position.x;
    double dy = target_pose.position.y - current_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double target_yaw = std::atan2(dy, dx);
    double robot_yaw = getYawFromQuaternion(current_pose_.orientation);
    double yaw_error = target_yaw - robot_yaw;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::min(0.5, 0.5 * distance);
    cmd.angular.z = std::min(0.5, 1.0 * yaw_error);

    cmd_pub_->publish(cmd);

    RCLCPP_INFO(this->get_logger(), "Driving to (%.2f, %.2f), distance: %.2f, yaw error: %.2f",
                target_pose.position.x, target_pose.position.y, distance, yaw_error);
  }

  void followPath() {
    if (current_index_ >= path_.size()) {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Path completed. Robot stopped.");
      current_index_ = 0;
      return;
    }
  
    const auto& target_pose = path_[current_index_].pose;
    double dx = target_pose.position.x - current_pose_.position.x;
    double dy = target_pose.position.y - current_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
  
    if (distance < 0.01) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu at (%.2f, %.2f).", current_index_,
                  target_pose.position.x, target_pose.position.y);
      current_index_++;
      stopRobot();
      return;
    }
  
    // Orientation control
    double target_yaw = std::atan2(dy, dx);
    double robot_yaw = getYawFromQuaternion(current_pose_.orientation);
    double yaw_error = target_yaw - robot_yaw;
  
    // Normalize yaw error to [-pi, pi]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
  
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::min(0.5, 0.5 * distance);         // P-controller for linear speed
    cmd.angular.z = std::min(0.5, 1.0 * yaw_error);       // P-controller for angular speed
  
    cmd_pub_->publish(cmd);
  
    RCLCPP_INFO(this->get_logger(),
                "Following path to (%.2f, %.2f), distance: %.2f, yaw error: %.2f, linear.x: %.2f, angular.z: %.2f",
                target_pose.position.x, target_pose.position.y, distance, yaw_error,
                cmd.linear.x, cmd.angular.z);
  }


  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  bool reachedTarget(const geometry_msgs::msg::Pose& target_pose, double threshold = 0.01) {
      double dx = target_pose.position.x - current_pose_.position.x;
      double dy = target_pose.position.y - current_pose_.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
  
      RCLCPP_INFO(this->get_logger(),
          "Target check:\n"
          "  -> target_pose: x=%.3f, y=%.3f\n"
          "  -> current_goal_pose: x=%.3f, y=%.3f\n"
          "  -> dx=%.3f, dy=%.3f, dist=%.3f\n"
          "  -> threshold=%.3f → %s",
          target_pose.position.x, target_pose.position.y,
          current_pose_.position.x, current_pose_.position.y,
          dx, dy, dist,
          threshold,
          dist < threshold ? "REACHED " : "NOT REACHED "
      );
  
      return dist < threshold;
  }



  enum class State { SEARCHING, APPROACHING, COLLECTING, RETURNING };
  State state_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<geometry_msgs::msg::Pose> nuts_;
  geometry_msgs::msg::Pose current_goal_pose_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose start_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_index_;
  rclcpp::Time collect_start_time_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
