// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// Software License Agreement (BSD License 2.0)
//
// C++ port of teleop_keyboard.py for Monicar2

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

static volatile std::sig_atomic_t g_stop = 0;
static void onSigint(int) {g_stop = 1;}

static constexpr int MAX_SONG = 5;
static constexpr int MAX_ANIM = 3;
static constexpr int MAX_COLOR = 6;

static const char * MSG = R"(
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

c: Change led
z: play buzzer song
p: OLED animation

CTRL-C to quit
)";

// Read one key with 0.1 s timeout, in raw mode. Returns 0 when no key.
static char getKey(const termios & saved)
{
  termios raw = saved;
  cfmakeraw(&raw);
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(STDIN_FILENO, &rfds);
  timeval tv{0, 100000};
  char key = 0;
  if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) > 0) {
    if (read(STDIN_FILENO, &key, 1) != 1) {
      key = 0;
    }
  }
  tcsetattr(STDIN_FILENO, TCSADRAIN, &saved);
  return key;
}

static double makeSimpleProfile(double output, double input, double slop)
{
  if (input > output) {
    return std::min(input, output + slop);
  } else if (input < output) {
    return std::max(input, output - slop);
  }
  return input;
}

static void printVels(double lin, double ang)
{
  std::printf("currently:\tlinear velocity %.3f\t angular velocity %.3f \n", lin, ang);
}

int main(int argc, char ** argv)
{
  termios saved{};
  const bool is_tty = (tcgetattr(STDIN_FILENO, &saved) == 0);
  if (!is_tty) {
    std::fprintf(stderr, "teleop_keyboard: stdin is not a terminal\n");
    return 1;
  }

  rclcpp::init(argc, argv);
  // own SIGINT handler so the stop command can still be published on CTRL-C
  std::signal(SIGINT, onSigint);
  auto node = rclcpp::Node::make_shared("teleop_keyboard_node");

  // Defaults match the original Python constants; overridable by parameters.
  const double MAX_LIN_VEL = node->declare_parameter<double>("MAX_LIN_VEL", 0.2);
  const double MAX_ANG_VEL = node->declare_parameter<double>("MAX_ANG_VEL", 0.8);
  const double LIN_VEL_STEP_SIZE = node->declare_parameter<double>("LIN_VEL_STEP_SIZE", 0.03);
  const double ANG_VEL_STEP_SIZE = node->declare_parameter<double>("ANG_VEL_STEP_SIZE", 0.1);

  std::printf(
    "Param max lin: %.3f m/s, max ang: %.3f rad/s, lin step: %.3f m/s ang step: %.3f rad/s\n",
    MAX_LIN_VEL, MAX_ANG_VEL, LIN_VEL_STEP_SIZE, ANG_VEL_STEP_SIZE);

  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
  auto ledpub = node->create_publisher<std_msgs::msg::Int32>("ledSub", 10);
  auto songpub = node->create_publisher<std_msgs::msg::Int32>("songSub", 10);
  auto lcdpub = node->create_publisher<std_msgs::msg::Int32>("lcdSub", 10);

  std::printf("Monicar Teleop Keyboard controller\n");

  int status = 0;
  double target_lin = 0.0, target_ang = 0.0;
  double control_lin = 0.0, control_ang = 0.0;
  int color_idx = 0, song_idx = 0, lcd_idx = 0;

  auto publishIdx = [](auto & p, const char * name, int & idx, int max_idx) {
      std::printf("%s: %d\n", name, idx);
      std_msgs::msg::Int32 m;
      m.data = idx;
      p->publish(m);
      if (++idx >= max_idx) {
        idx = 0;
      }
    };

  std::printf("%s", MSG);
  while (rclcpp::ok() && !g_stop) {
    const char key = getKey(saved);
    if (key == 'w') {
      target_lin = std::clamp(target_lin + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL);
      ++status;
      printVels(target_lin, target_ang);
    } else if (key == 'x') {
      target_lin = std::clamp(target_lin - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL);
      ++status;
      printVels(target_lin, target_ang);
    } else if (key == 'a') {
      target_ang = std::clamp(target_ang + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL);
      ++status;
      printVels(target_lin, target_ang);
    } else if (key == 'd') {
      target_ang = std::clamp(target_ang - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL);
      ++status;
      printVels(target_lin, target_ang);
    } else if (key == ' ' || key == 's') {
      target_lin = control_lin = 0.0;
      target_ang = control_ang = 0.0;
      printVels(target_lin, target_ang);
    } else if (key == 'c') {
      publishIdx(ledpub, "colorIdx", color_idx, MAX_COLOR);
    } else if (key == 'z') {
      publishIdx(songpub, "songIdx", song_idx, MAX_SONG);
    } else if (key == 'p') {
      publishIdx(lcdpub, "lcdIdx", lcd_idx, MAX_ANIM);
    } else if (key == '\x03') {  // CTRL-C (raw mode)
      break;
    }

    if (status == 20) {
      std::printf("%s", MSG);
      status = 0;
    }

    geometry_msgs::msg::Twist twist;
    control_lin = makeSimpleProfile(control_lin, target_lin, LIN_VEL_STEP_SIZE / 2.0);
    control_ang = makeSimpleProfile(control_ang, target_ang, ANG_VEL_STEP_SIZE / 2.0);
    twist.linear.x = control_lin;
    twist.angular.z = control_ang;
    pub->publish(twist);
  }

  // stop the robot
  if (rclcpp::ok()) {
    pub->publish(geometry_msgs::msg::Twist());
  }
  tcsetattr(STDIN_FILENO, TCSADRAIN, &saved);
  rclcpp::shutdown();
  return 0;
}
