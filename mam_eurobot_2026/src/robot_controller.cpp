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
    planner_ = std::make_shared<TrajectoryPlanner>(this->shared_from_this());

    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/supposed_objects", 10, std::bind(&RobotController::markerCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&RobotController::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(1s, std::bind(&RobotController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "RobotController node initialized.");
  }

private:
  void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    for (const auto& marker : msg->markers) {
      planner_->addMarker(marker);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    if (state_ == State::SEARCHING && start_pose_.position.x == 0.0 && start_pose_.position.y == 0.0) {
      start_pose_ = current_pose_;
    }
    
  }

  void controlLoop() {
    bool reached = false;
    
    switch (state_) {
      case State::SEARCHING:
        RCLCPP_INFO(this->get_logger(), "[SEARCHING] Checking for available markers...");
        if (planner_->setGoal()) {
          state_ = State::APPROACHING;
        } else {
          discorver();
        }
        break;

      case State::APPROACHING:
        RCLCPP_INFO(this->get_logger(), "[APPROACHING] Driving to target...");
        path_ = planner_->planTrajectory(reached);

        if (reached) {
          stopRobot();
          collect_start_time_ = this->now();
          state_ = State::COLLECTING;
        } else {
          followPath();
        }
        break;

      case State::COLLECTING:
        RCLCPP_INFO(this->get_logger(), "[COLLECTING] Waiting...");
        if ((this->now() - collect_start_time_).seconds() > 2.0) {
          planner_->setHomePosition(start_pose_);
          path_ = planner_->planWayHome();
          state_ = State::RETURNING;
        }
        break;

      case State::RETURNING:
        RCLCPP_INFO(this->get_logger(), "[RETURNING] Driving to nest...");
        if (reachedTarget(start_pose_)) {
          stopRobot();
          state_ = State::SEARCHING;
        } else {
          path_ = planner_->planWayHome();
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
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void discorver() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.3;
    cmd.angular.z = 0.2;
    cmd_pub_->publish(cmd);

  }

  void followPath() {
    if (current_index_ >= path_.size()) {
      stopRobot();
      current_index_ = 0;
      return;
    }

    const auto& target_pose = path_[current_index_].pose;
    double dx = target_pose.position.x - current_pose_.position.x;
    double dy = target_pose.position.y - current_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 0.01) {
      current_index_++;
      return;
    }

    double target_yaw = std::atan2(dy, dx);
    double robot_yaw = getYawFromQuaternion(current_pose_.orientation);
    double yaw_error = target_yaw - robot_yaw;

    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::min(0.5, 0.5 * distance);
    cmd.angular.z = std::min(0.5, 1.0 * yaw_error);
    cmd_pub_->publish(cmd);
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
    return dist < threshold;
  }

  enum class State { SEARCHING, APPROACHING, COLLECTING, RETURNING };
  State state_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<TrajectoryPlanner> planner_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose current_goal_pose_;
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