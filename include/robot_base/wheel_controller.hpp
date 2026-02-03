#ifndef ROBOT_BASE__WHEEL_CONTROLLER_HPP_
#define ROBOT_BASE__WHEEL_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cstdint>
#include <string>

class WheelController : public rclcpp::Node {
public:
  WheelController();
  ~WheelController() override;
  
private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();
  void updateCommandFromTwist(double linear, double angular);
  void sendMotorCommand(uint8_t speed, uint8_t left_dir, uint8_t right_dir);
  void readSerialResponses();
  bool openSerialPort();
  void closeSerialPort();
  uint8_t directionFromSpeed(double speed) const;
  uint8_t speedToPwm(double speed) const;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  
  std::string serial_port_;
  int baud_rate_;
  double wheel_distance_;
  double max_linear_speed_;
  double deadband_;
  double send_rate_hz_;
  int cmd_timeout_ms_;

  int serial_fd_;

  rclcpp::Time last_cmd_time_;
  bool has_cmd_;
  uint8_t last_speed_;
  uint8_t last_left_dir_;
  uint8_t last_right_dir_;
};

#endif  // ROBOT_BASE__WHEEL_CONTROLLER_HPP_
