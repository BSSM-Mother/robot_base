#ifndef ROBOT_BASE__WHEEL_CONTROLLER_HPP_
#define ROBOT_BASE__WHEEL_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WheelController : public rclcpp::Node {
public:
  WheelController();
  
private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  float left_wheel_speed_;
  float right_wheel_speed_;
};

#endif  // ROBOT_BASE__WHEEL_CONTROLLER_HPP_
