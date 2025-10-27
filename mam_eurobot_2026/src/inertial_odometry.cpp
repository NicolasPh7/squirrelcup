#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <mutex>
#include <cmath>

class InertialOdometry : public rclcpp::Node
{
public:
  InertialOdometry()
  : Node("inertial_odometry"), publish_frequency_(1000)
  {
    this->declare_parameter("publish_frequency", publish_frequency_);
    this->get_parameter("publish_frequency", publish_frequency_);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&InertialOdometry::cmdVelCallback, this, std::placeholders::_1));

    poseCorrectionSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/corrected_pose", 10, std::bind(&InertialOdometry::correctedPoseCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  rclcpp::Time now = this->now();    last_time_ = now;

    // TF broadcaster
    // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&InertialOdometry::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(), "InertialOdometry started, publishing /odom at %.1f Hz",
      publish_frequency_);
  }

private:
  void correctedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Convert poses to tf2::Transform
    tf2::Transform corrected_tf, odom_tf;
    tf2::fromMsg(msg->pose, corrected_tf);
    tf2::fromMsg(last_odom_.pose.pose, odom_tf);  

    // Compute T = corrected_pose * inverse(odom_pose)
    tf2::Transform T = corrected_tf * odom_tf.inverse();

    // Store T for future use
    // transform_odom_to_map_ = T;

    // Optionally apply T to current odometry
    tf2::Transform corrected_pose = T * odom_tf;

    // Extract position and yaw
    x_ = corrected_pose.getOrigin().x();
    y_ = corrected_pose.getOrigin().y();
    tf2::Quaternion q = corrected_pose.getRotation();
    yaw_ = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }


  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    omega_ = msg->angular.z;
  }

  void timerCallback()
  {
  rclcpp::Time now = this->now();    double dt = (now - last_time_).seconds();
    last_time_ = now;

    double local_vx = 0.0, local_omega = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_vx = vx_;
      local_omega = omega_;
    }

    // Runge-Kutta 2. Ordnung
    double mid_yaw = yaw_ + 0.5 * local_omega * dt;
    double dx = local_vx * std::cos(mid_yaw) * dt;
    double dy = local_vx * std::sin(mid_yaw) * dt;
    double dyaw = local_omega * dt;

    x_ += dx;
    y_ += dy;
    yaw_ += dyaw;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));  

    // publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = local_vx;
    odom.twist.twist.angular.z = local_omega;
    pub_->publish(odom);
    last_odom_ = odom;

    // // publish TF odom -> base_link
    // geometry_msgs::msg::TransformStamped t;
    // t.header.stamp = now;
    // t.header.frame_id = "odom";
    // t.child_frame_id = "base_link";
    // t.transform.translation.x = x_;
    // t.transform.translation.y = y_;
    // t.transform.translation.z = 0.0;
    // tf2::Quaternion qt;
    // qt.setRPY(0.0, 0.0, yaw_);
    // t.transform.rotation.x = qt.x();
    // t.transform.rotation.y = qt.y();
    // t.transform.rotation.z = qt.z();
    // t.transform.rotation.w = qt.w();
    // tf_broadcaster_->sendTransform(t);
  }


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseCorrectionSub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
  double vx_ = 0.0;
  double vy_ = 0.0;
  double omega_ = 0.0;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_time_;
  double publish_frequency_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InertialOdometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
