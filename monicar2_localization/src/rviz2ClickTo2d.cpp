// Author: ChangeWhan Lee
// C++ port of rviz2ClickTo2d.py / rviz2ClickTo2dTimer.py
//  - goal_pose (PoseStamped)                -> goal_2d (PoseStamped, orientation.z = yaw)
//  - initialpose (PoseWithCovarianceStamped) -> initial_2d (PoseStamped, orientation.z = yaw)
//
// Built twice:
//  - rviz2ClickTo2d      : publishes immediately on click
//  - rviz2ClickTo2dTimer : (USE_TIMER) latches the click and publishes from a 200 ms timer

#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// yaw of quaternion (0, 0, z, w), same as euler_from_quaternion() in the Python version
static double yawFromZW(double z, double w)
{
  const double siny_cosp = 2.0 * (w * z);
  const double cosy_cosp = 1.0 - 2.0 * (z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

static geometry_msgs::msg::PoseStamped toRpyPose(
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = "map";
  out.header.stamp = stamp;
  out.pose.position.x = pose.position.x;
  out.pose.position.y = pose.position.y;
  out.pose.position.z = 0.0;
  out.pose.orientation.x = 0.0;
  out.pose.orientation.y = 0.0;
  out.pose.orientation.z = yawFromZW(pose.orientation.z, pose.orientation.w);
  out.pose.orientation.w = 0.0;
  return out;
}

class Rviz2ClickTo2d : public rclcpp::Node
{
public:
  Rviz2ClickTo2d()
  : Node("rviz2_click_node")
  {
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_2d", 10);
    init_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("initial_2d", 10);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "goal clicked");
        rpy_goal_ = toRpyPose(msg->header.stamp, msg->pose);
#ifdef USE_TIMER
        goal_clicked_ = true;
#else
        goal_pub_->publish(rpy_goal_);
#endif
      });

    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "initial pose clicked");
        rpy_pose_ = toRpyPose(msg->header.stamp, msg->pose.pose);
#ifdef USE_TIMER
        init_clicked_ = true;
#else
        init_pub_->publish(rpy_pose_);
#endif
      });

#ifdef USE_TIMER
    timer_ = create_wall_timer(200ms, [this]() {
          if (goal_clicked_) {
            goal_pub_->publish(rpy_goal_);
            goal_clicked_ = false;
          }
          if (init_clicked_) {
            init_pub_->publish(rpy_pose_);
            init_clicked_ = false;
          }
        });
#endif
    RCLCPP_INFO(get_logger(), "Click2To2d done");
  }

private:
  geometry_msgs::msg::PoseStamped rpy_goal_, rpy_pose_;
  bool goal_clicked_ = false;
  bool init_clicked_ = false;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_, init_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rviz2ClickTo2d>());
  rclcpp::shutdown();
  return 0;
}
