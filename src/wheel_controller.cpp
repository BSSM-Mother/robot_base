#include "robot_base/wheel_controller.hpp"

WheelController::WheelController() : rclcpp::Node("wheel_controller") {
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&WheelController::cmdVelCallback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(this->get_logger(), "Wheel Controller initialized");
}

void WheelController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // 원통형 로봇을 위한 바퀴 속도 계산
  // v = linear.x, w = angular.z
  float v = msg->linear.x;
  float w = msg->angular.z;
  float wheel_distance = 0.25;  // 바퀴 간 거리
  
  // 양쪽 바퀴 속도 계산 (차분 구동)
  left_wheel_speed_ = v + (w * wheel_distance / 2);
  right_wheel_speed_ = v - (w * wheel_distance / 2);
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Left: %.2f, Right: %.2f", left_wheel_speed_, right_wheel_speed_);
  
  // TODO: 실제 하드웨어 제어 코드 추가
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}
