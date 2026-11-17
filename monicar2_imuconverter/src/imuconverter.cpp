// Author: ChangeWhan Lee
// C++ port of imuconverter.py
//  - subscribe /quaternion (geometry_msgs/Quaternion)
//  - publish /imu/data (sensor_msgs/Imu)
//  - broadcast TF: base_footprint->base_link, base_link->laser_link, base_link->imu_link

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_node")
  {
    timer_tick_ = declare_parameter<double>("timer_tick", 0.05);
    use_imu_ = declare_parameter<bool>("useImu", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

    RCLCPP_INFO(get_logger(), "Setting Up the Node...");
    RCLCPP_INFO(get_logger(), "timer_tick: %.3f sec, useImu: %s",
      timer_tick_, use_imu_ ? "true" : "false");

    quat_sub_ = create_subscription<geometry_msgs::msg::Quaternion>(
      "quaternion", 10,
      [this](const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        imu_msg_.orientation = *msg;
      });
    RCLCPP_INFO(get_logger(), "quaternion subscriber set");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    RCLCPP_INFO(get_logger(), "Imu publisher set");

    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const double linear_acceleration_stdev = 0.06;
    const double angular_velocity_stdev = 0.005;
    const double linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev;
    const double angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev;

    // See imuconverter.py history for the covariance estimation rationale.
    imu_msg_.orientation_covariance = {
      0.1, 0.0, 0.0,
      0.0, 0.1, 0.0,
      0.0, 0.0, 0.1};
    imu_msg_.angular_velocity_covariance = {
      0.0, 0.0, 0.0,
      0.0, angular_velocity_cov, 0.0,
      0.0, 0.0, angular_velocity_cov};
    imu_msg_.linear_acceleration_covariance = {
      0.0, 0.0, 0.0,
      0.0, linear_acceleration_cov, 0.0,
      0.0, 0.0, linear_acceleration_cov};

    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_tick_), std::bind(&ImuNode::cbTimer, this));
  }

private:
  void sendTf(
    const rclcpp::Time & stamp, const std::string & parent, const std::string & child,
    double x, double y, double z, const geometry_msgs::msg::Quaternion * rot = nullptr)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    if (rot) {
      t.transform.rotation = *rot;
    }  // else identity (default w=1)
    br_->sendTransform(t);
  }

  void cbTimer()
  {
    const rclcpp::Time now = get_clock()->now();
    imu_msg_.header.frame_id = frame_id_;
    imu_msg_.header.stamp = now;
    imu_pub_->publish(imu_msg_);

    // static transforms
    sendTf(now, "base_footprint", "base_link", 0.0, 0.0, 0.015);
    sendTf(now, "base_link", "laser_link", 0.065, 0.0, 0.13);
    // relative position, base_link -> imu_link, rotated by imu orientation
    sendTf(now, "base_link", "imu_link", 0.065, 0.0, 0.06, &imu_msg_.orientation);
  }

  double timer_tick_;
  bool use_imu_;
  std::string frame_id_;
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quat_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
