// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// Software License Agreement (BSD License 2.0)
//
// C++ port of teleop_joy.py for Monicar2

#include <chrono>
#include <cstdio>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

static constexpr int MAX_SONG = 5;
static constexpr int MAX_ANIM = 3;
static constexpr int MAX_COLOR = 6;
static constexpr int MAX_CHAT = 5;

static const char * MSG = R"(
Control Your Robot!
---------------------------
'A' : Change led
'B' : Play buzzer song
'Y': Play OLED animation
)";

class TeleopJoyNode : public rclcpp::Node
{
public:
  TeleopJoyNode()
  : Node("teleop_joy_node")
  {
    max_fwd_vel_ = declare_parameter<double>("max_fwd_m_s", 0.2);
    max_rev_vel_ = declare_parameter<double>("max_rev_m_s", 0.2);
    max_ang_vel_ = declare_parameter<double>("max_rad_s", 0.2);

    pub_led_ = create_publisher<std_msgs::msg::Int32>("ledSub", 10);
    pub_song_ = create_publisher<std_msgs::msg::Int32>("songSub", 10);
    pub_lcd_ = create_publisher<std_msgs::msg::Int32>("lcdSub", 10);
    pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));

    std::printf(" Monicar Teleop Joystick controller\n%s\n", MSG);
    std::printf("Param max fwd: %.3f m/s, max rev: -%.3f m/s, max ang: %.3f rad/s\n",
      max_fwd_vel_, max_rev_vel_, max_ang_vel_);
    std::printf("CTRL-C to quit\n");

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&TeleopJoyNode::cbJoy, this, std::placeholders::_1));
    timer_ = create_wall_timer(100ms, std::bind(&TeleopJoyNode::cbTimer, this));
  }

private:
  void publishIdx(
    const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr & pub,
    const char * name, int & idx, int max_idx)
  {
    std::printf("%s: %d\n", name, idx);
    std_msgs::msg::Int32 m;
    m.data = idx;
    pub->publish(m);
    if (++idx >= max_idx) {
      idx = 0;
    }
    mode_button_last_ = 1;
  }

  void cbJoy(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    if (joy->axes.size() < 3 || joy->buttons.size() < 5) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Joy message too short (axes=%zu, buttons=%zu)", joy->axes.size(), joy->buttons.size());
      return;
    }

    const double ax = joy->axes[1];
    if (ax > 0.0) {
      twist_.linear.x = ax * max_fwd_vel_;
    } else if (ax < 0.0) {
      twist_.linear.x = ax * max_rev_vel_;
    } else {
      twist_.linear.x = 0.0;
    }

    if (mode_button_last_ == 0) {
      if (joy->buttons[0] == 1) {
        publishIdx(pub_led_, "colorIdx", color_idx_, MAX_COLOR);
      } else if (joy->buttons[1] == 1) {
        publishIdx(pub_song_, "songIdx", song_idx_, MAX_SONG);
      } else if (joy->buttons[4] == 1) {
        publishIdx(pub_lcd_, "lcdIdx", lcd_idx_, MAX_ANIM);
      }
    }

    twist_.linear.y = 0.0;
    twist_.linear.z = 0.0;
    twist_.angular.x = 0.0;
    twist_.angular.y = 0.0;
    twist_.angular.z = joy->axes[2] * max_ang_vel_;
  }

  void cbTimer()
  {
    pub_twist_->publish(twist_);
    // protect chattering
    if (++chat_count_ > MAX_CHAT) {
      mode_button_last_ = 0;
      chat_count_ = 0;
    }
  }

  double max_fwd_vel_, max_rev_vel_, max_ang_vel_;
  int chat_count_ = 0;
  int mode_button_last_ = 0;
  int color_idx_ = 0;
  int song_idx_ = 0;
  int lcd_idx_ = 0;
  geometry_msgs::msg::Twist twist_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_led_, pub_song_, pub_lcd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopJoyNode>());
  rclcpp::shutdown();
  return 0;
}
