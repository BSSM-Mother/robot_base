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
  void sendMotorCommand(uint8_t rspeed, uint8_t lspeed, uint8_t rdir, uint8_t ldir);
  void readSerialResponses();
  bool openSerialPort();
  void closeSerialPort();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;

  std::string serial_port_;
  int baud_rate_;
  double wheel_distance_;
  double max_linear_speed_;
  double deadband_;
  double send_rate_hz_;
  int cmd_timeout_ms_;
  int min_pwm_;

  int serial_fd_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time kickstart_time_;
  bool has_cmd_;
  uint8_t last_left_pwm_;
  uint8_t last_right_pwm_;
  uint8_t last_left_dir_;
  uint8_t last_right_dir_;

  // 직전 전송값 기억 → 중복 전송 방지
  uint8_t last_sent_rspeed_;
  uint8_t last_sent_lspeed_;
  uint8_t last_sent_rdir_;
  uint8_t last_sent_ldir_;

  // 정지→움직임 전환 시 킥스타트 부스트
  static constexpr int     KICKSTART_DURATION_MS_ = 200;
  static constexpr uint8_t KICKSTART_PWM_         = 150;

  // 움직임→정지 전환 시 역방향 브레이크
  rclcpp::Time brake_time_;
  bool         is_braking_;
  uint8_t      brake_left_dir_;
  uint8_t      brake_right_dir_;
  static constexpr int     BRAKE_DURATION_MS_ = 80;
  static constexpr uint8_t BRAKE_PWM_         = 180;
};

#endif  // ROBOT_BASE__WHEEL_CONTROLLER_HPP_
