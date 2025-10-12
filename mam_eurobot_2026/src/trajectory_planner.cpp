#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TrajectoryPlanner : public rclcpp::Node {
public:
  TrajectoryPlanner() : Node("trajectory_planner") {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    goal_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
      "/current_goal_marker", 10, std::bind(&TrajectoryPlanner::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&TrajectoryPlanner::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TrajectoryPlanner node started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    has_odom_ = true;
  }

  void goalCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    goal_pose_ = msg->pose;
    has_goal_ = true;

    if (has_odom_) {
      planPath();
    }
  }

  void planPath() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map"; 

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = path_msg.header;
    start_pose.pose.position = current_position_;
    start_pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(start_pose);

    double dx = goal_pose_.position.x - current_position_.x;
    double dy = goal_pose_.position.y - current_position_.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double offset = 0.01;  

    double target_x = goal_pose_.position.x - offset * (dx / distance);
    double target_y = goal_pose_.position.y - offset * (dy / distance);

    int num_poses = 5;
    for (int i = 1; i <= num_poses; ++i) {
      double ratio = static_cast<double>(i) / num_poses;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = current_position_.x + ratio * (target_x - current_position_.x);
      pose.pose.position.y = current_position_.y + ratio * (target_y - current_position_.y);
      pose.pose.orientation = goal_pose_.orientation;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses toward goal at (%.2f, %.2f).",
                path_msg.poses.size(), goal_pose_.position.x, goal_pose_.position.y);

    has_odom_ = false;
    has_goal_ = false;

  }


  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Pose goal_pose_;
  bool has_goal_ = false;
  bool has_odom_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlanner>());
  rclcpp::shutdown();
  return 0;
}
