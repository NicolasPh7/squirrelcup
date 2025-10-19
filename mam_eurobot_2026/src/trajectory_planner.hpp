#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>

class TrajectoryPlanner {
public:
  TrajectoryPlanner(rclcpp::Node::SharedPtr node)
    : node_(node),
      path_pub_(node_->create_publisher<nav_msgs::msg::Path>("/planned_path", 10)) {}

  void setHomePosition(const geometry_msgs::msg::Pose& home) {
    home_position_ = home;
    has_home_ = true;
  }

  void updateCurrentPosition(const geometry_msgs::msg::Pose& position) {
    current_position_ = position;
    has_position_ = true;
  }

  void clearMarkers() {
    markers_.clear();
  }

  void addMarker(const visualization_msgs::msg::Marker& marker) {
    for (const auto& existing : markers_) {
      double dx = existing.pose.position.x - marker.pose.position.x;
      double dy = existing.pose.position.y - marker.pose.position.y;
      double dz = existing.pose.position.z - marker.pose.position.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < 0.01) return;  
    }
  
    if (has_home_) {
      double dx_home = home_position_.position.x - marker.pose.position.x;
      double dy_home = home_position_.position.y - marker.pose.position.y;
      double dz_home = home_position_.position.z - marker.pose.position.z;
      double dist_home = std::sqrt(dx_home * dx_home + dy_home * dy_home + dz_home * dz_home);
      if (dist_home < 0.4) return;  
    }
  
    markers_.push_back(marker);
  }

  bool setGoal() {
    visualization_msgs::msg::Marker marker;

    for (const auto& existing : markers_) {
        if (existing.color.b == 1.0 && existing.color.g == 0.0) {
          marker = existing;
          has_goal_ = true;
          break;
       }
        if (existing.color.b == 0.0 && existing.color.g == 1.0 && existing.color.r == 1.0) {
          marker = existing;
          has_goal_ = true;
          break;
        }
        if (existing.color.b == 0.5) {
          marker = existing;
          has_goal_ = true;
          break;
        }
    }    
    if (!has_goal_) return false;

    goal_pose_ = marker.pose;
    return true;
  }

  std::vector<geometry_msgs::msg::PoseStamped> planTrajectory(bool& goalReached) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = node_->now();
    path_msg.header.frame_id = "base_link";

    if (!has_goal_) return path_msg.poses;
    if (IsGoalReached()) goalReached = true;

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = path_msg.header;
    start_pose.pose = current_position_;
    path_msg.poses.push_back(start_pose);

    geometry_msgs::msg::Pose target = goal_pose_;
    generatePath(path_msg, current_position_, target);

    path_pub_->publish(path_msg);
    RCLCPP_INFO(node_->get_logger(), "Published path to goal.");

    return path_msg.poses;
  }

  std::vector<geometry_msgs::msg::PoseStamped> planWayHome() {
    nav_msgs::msg::Path return_path;
    return_path.header.stamp = node_->now();
    return_path.header.frame_id = "base_link";
    
    if (has_home_ && has_position_) {
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.header = return_path.header;
      start_pose.pose = current_position_;
      return_path.poses.push_back(start_pose);
      generatePath(return_path, start_pose.pose, home_position_);
      path_pub_->publish(return_path);
      RCLCPP_INFO(node_->get_logger(), "Published return path to home.");

    }
    return return_path.poses;
  }

  bool IsGoalReached() {
    const auto& marker = goal_pose_;
    double dx = marker.position.x;
    double dy = marker.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 0.01) { 
      for (size_t i = 0; i < markers_.size(); i++) {
        double dx = goal_pose_.position.x - markers_[i].pose.position.x;
        double dy = goal_pose_.position.y - markers_[i].pose.position.y;
        double dz = goal_pose_.position.z - markers_[i].pose.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 0.01){ markers_.erase( markers_.begin() + i ); break;}  
         
      }
      return true;
    }
    return false;
  }

private:
  void generatePath(nav_msgs::msg::Path& path_msg,
                    const geometry_msgs::msg::Pose& start,
                    const geometry_msgs::msg::Pose& end) {
    int num_poses = 10;
    for (int i = 1; i <= num_poses; ++i) {
      double ratio = static_cast<double>(i) / num_poses;
      geometry_msgs::msg::Pose intermediate;
      intermediate.position.x = start.position.x + ratio * (end.position.x - start.position.x);
      intermediate.position.y = start.position.y + ratio * (end.position.y - start.position.y);

      if (isNearMarker(intermediate)) continue;

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose = intermediate;
      pose.pose.orientation.w = end.orientation.w;
      path_msg.poses.push_back(pose);
    }
  }

  bool isNearMarker(const geometry_msgs::msg::Pose& point) {
    for (const auto& marker : markers_) {
      double dx = marker.pose.position.x - point.position.x;
      double dy = marker.pose.position.y - point.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < 0.01) return true;
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  geometry_msgs::msg::Pose current_position_;
  geometry_msgs::msg::Pose home_position_;
  geometry_msgs::msg::Pose goal_pose_;
  std::vector<visualization_msgs::msg::Marker> markers_;

  bool has_position_ = false;
  bool has_goal_ = false;
  bool has_home_ = false;
};
