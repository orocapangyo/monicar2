// C++ port of move.py / rotate.py
// Built twice:
//  - move   : drive straight for a given distance
//  - rotate : (ROTATE) turn in place for a given angle
// Publishes cmd_vel at 10 Hz until the target is reached, then publishes a stop command.

#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

static volatile std::sig_atomic_t g_stop = 0;
static void onSigint(int) {g_stop = 1;}

template<typename T>
static bool ask(const char * prompt, T & out)
{
  std::cout << prompt << std::flush;
  return static_cast<bool>(std::cin >> out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // own SIGINT handler so the stop command can still be published on CTRL-C
  std::signal(SIGINT, onSigint);

#ifdef ROTATE
  auto node = rclcpp::Node::make_shared("rotate_node");
#else
  auto node = rclcpp::Node::make_shared("move_node");
#endif
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  geometry_msgs::msg::Twist vel;

  double speed = 0.0, target = 0.0;
  int dir = 0;
#ifdef ROTATE
  std::cout << "Let's rotate your robot" << std::endl;
  if (!ask("Input your speed (degrees/sec):", speed) ||
    !ask("Type your distance (degrees):", target) ||
    !ask("Clockwise, 0 or 1:", dir))
  {
    std::cerr << "invalid input" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  speed = speed * 2.0 * M_PI / 360.0;
  target = target * 2.0 * M_PI / 360.0;
  vel.angular.z = dir ? std::fabs(speed) : -std::fabs(speed);
  std::cout << "Start to rotate" << std::endl;
#else
  std::cout << "Let's move your robot" << std::endl;
  if (!ask("Input your speed, 0.0~0.2(m/s): ", speed) ||
    !ask("Type your distance, 0~1(meter): ", target) ||
    !ask("Foward, 0 or 1: ", dir))
  {
    std::cerr << "invalid input" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  vel.linear.x = (dir == 1) ? std::fabs(speed) : -std::fabs(speed);
  std::cout << "Start to move" << std::endl;
#endif

  const rclcpp::Time t0 = node->now();
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok() && !g_stop) {
    rate.sleep();
    pub->publish(vel);
    const double elapsed = (node->now() - t0).seconds();
    std::cout << "ct: " << elapsed << std::endl;
    if (speed * elapsed > target) {
      break;
    }
  }

  std::cout << "Stop to move" << std::endl;
  pub->publish(geometry_msgs::msg::Twist());
  rclcpp::shutdown();
  return 0;
}
