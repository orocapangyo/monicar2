// Author: ChangeWhan Lee
// C++ port of odomPublisher.py
//  - subscribe left_ticks / right_ticks (std_msgs/Int32), initial_2d (PoseStamped)
//  - publish odom (nav_msgs/Odometry) and TF odom -> base_footprint

#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/transform_broadcaster.h"

static constexpr double WHEELTRACK = 0.160;
static constexpr double WHEELRADIUS = 0.033;

class OdomNode : public rclcpp::Node
{
public:
  OdomNode()
  : Node("odompub_node")
  {
    init_pose_received_ = declare_parameter<bool>("initPoseRecieved", true);
    tpr_ = declare_parameter<double>("TPR", 1860.0);
    timer_tick_ = declare_parameter<double>("timer_tick", 0.05);
    RCLCPP_INFO(get_logger(), "initPoseRecieved: %s, TPR: %.1f, timer_tick: %.3f",
      init_pose_received_ ? "true" : "false", tpr_, timer_tick_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    left_sub_ = create_subscription<std_msgs::msg::Int32>(
      "left_ticks", 10, [this](const std_msgs::msg::Int32::SharedPtr m) {left_ticks_ = m->data;});
    right_sub_ = create_subscription<std_msgs::msg::Int32>(
      "right_ticks", 10, [this](const std_msgs::msg::Int32::SharedPtr m) {right_ticks_ = m->data;});
    init_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "initial_2d", 10, std::bind(&OdomNode::init2dCallback, this, std::placeholders::_1));

    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_tick_), std::bind(&OdomNode::cbTimer, this));
    RCLCPP_INFO(get_logger(), "odompub_node done");
  }

private:
  void init2dCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    x_ = msg->pose.position.x;
    y_ = msg->pose.position.y;
    init_pose_received_ = true;
    RCLCPP_INFO(get_logger(), "init2dCallback: true");
  }

  void cbTimer()
  {
    if (!init_pose_received_) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    const double dt = timer_tick_;

    const int delta_l = left_ticks_ - last_left_ticks_;
    const int delta_r = right_ticks_ - last_right_ticks_;
    const double dl = 2.0 * M_PI * WHEELRADIUS * delta_l / tpr_;
    const double dr = 2.0 * M_PI * WHEELRADIUS * delta_r / tpr_;

    const double dc = (dl + dr) / 2.0;
    const double dth = (dr - dl) / WHEELTRACK;

    double dx, dy;
    if (dr == dl) {
      dx = dr * std::cos(th_);
      dy = dr * std::sin(th_);
    } else {
      const double radius = dc / dth;
      const double icc_x = x_ - radius * std::sin(th_);
      const double icc_y = y_ + radius * std::cos(th_);
      dx = std::cos(dth) * (x_ - icc_x) - std::sin(dth) * (y_ - icc_y) + icc_x - x_;
      dy = std::sin(dth) * (x_ - icc_x) + std::cos(dth) * (y_ - icc_y) + icc_y - y_;
    }

    x_ += dx;
    y_ += dy;
    // Python's % is always non-negative; emulate it
    th_ = std::fmod(th_ + dth, 2.0 * M_PI);
    if (th_ < 0.0) {
      th_ += 2.0 * M_PI;
    }

    const double vx = dx / dt;
    const double vth = dth / dt;

    // quaternion from yaw only (roll = pitch = 0)
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.header.stamp = now;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(th_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(th_ * 0.5);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_br_->sendTransform(tf);

    last_left_ticks_ = left_ticks_;
    last_right_ticks_ = right_ticks_;
  }

  bool init_pose_received_;
  double tpr_, timer_tick_;
  int left_ticks_ = 0, right_ticks_ = 0;
  int last_left_ticks_ = 0, last_right_ticks_ = 0;
  double x_ = 0.0, y_ = 0.0, th_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_sub_, right_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
