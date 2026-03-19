#include "robot_base/wheel_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {
constexpr uint8_t kSpeedStop = 0;
constexpr uint8_t kDirStop   = 0;
// 왼쪽/오른쪽 모터가 반대로 장착되어 방향 코드가 서로 엇갈림
constexpr uint8_t kLeftFwd  = 1;
constexpr uint8_t kLeftRev  = 2;
constexpr uint8_t kRightFwd = 2;
constexpr uint8_t kRightRev = 1;

speed_t toTermiosBaud(int baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
    default:
      return B115200;
  }
}
}  // namespace

WheelController::WheelController()
    : rclcpp::Node("wheel_controller"),
      serial_port_(this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0")),
      baud_rate_(this->declare_parameter<int>("baud_rate", 115200)),
      wheel_distance_(this->declare_parameter<double>("wheel_distance", 0.25)),
      max_linear_speed_(this->declare_parameter<double>("max_linear_speed", 0.5)),
      deadband_(this->declare_parameter<double>("deadband", 0.02)),
      send_rate_hz_(this->declare_parameter<double>("send_rate_hz", 50.0)),
      cmd_timeout_ms_(this->declare_parameter<int>("cmd_timeout_ms", 300)),
      min_pwm_(this->declare_parameter<int>("min_pwm", 120)),
      serial_fd_(-1),
      last_cmd_time_(this->now()),
      kickstart_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      has_cmd_(false),
      last_speed_(0),
      last_left_dir_(kDirStop),
      last_right_dir_(kDirStop),
      brake_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      is_braking_(false),
      brake_left_dir_(kDirStop),
      brake_right_dir_(kDirStop),
      last_sent_speed_(255),  // 255: 불가능한 값 → 첫 전송 시 반드시 전송
      last_sent_left_(255),
      last_sent_right_(255) {
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&WheelController::cmdVelCallback, this, std::placeholders::_1));

  if (!openSerialPort()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
  }

  auto period = std::chrono::duration<double>(1.0 / std::max(1.0, send_rate_hz_));
  send_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&WheelController::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Wheel Controller initialized");
}

WheelController::~WheelController() {
  // 노드 종료 시 정지 명령을 여러 번 전송
  if (serial_fd_ >= 0) {
    for (int i = 0; i < 5; ++i) {
      sendMotorCommand(kSpeedStop, kDirStop, kDirStop);
    }
    ::tcdrain(serial_fd_);  // 버퍼 flush 후 닫기
  }
  closeSerialPort();
}

void WheelController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  updateCommandFromTwist(msg->linear.x, msg->angular.z);
  sendMotorCommand(last_speed_, last_left_dir_, last_right_dir_);
}

void WheelController::timerCallback() {
  const auto now = this->now();
  if (has_cmd_ && (now - last_cmd_time_).nanoseconds() >
                    static_cast<int64_t>(cmd_timeout_ms_) * 1000000LL) {
    RCLCPP_INFO(this->get_logger(),
      "[TIMEOUT] cmd_vel 없음 → 정지 (last_speed=%u, elapsed=%.0fms)",
      last_speed_, (now - last_cmd_time_).nanoseconds() / 1e6);
    last_speed_     = kSpeedStop;
    last_left_dir_  = kDirStop;
    last_right_dir_ = kDirStop;
    has_cmd_ = false;
  }

  sendMotorCommand(last_speed_, last_left_dir_, last_right_dir_);
  readSerialResponses();
}

void WheelController::updateCommandFromTwist(double linear, double angular) {
  const double half_track = wheel_distance_ / 2.0;
  const double left_speed = linear + (angular * half_track);
  const double right_speed = linear - (angular * half_track);

  const uint8_t left_dir = (std::abs(left_speed) <= deadband_) ? kDirStop :
                           (left_speed >= 0.0) ? kLeftFwd : kLeftRev;
  const uint8_t right_dir = (std::abs(right_speed) <= deadband_) ? kDirStop :
                            (right_speed >= 0.0) ? kRightFwd : kRightRev;

  const double max_mag = std::max(std::abs(left_speed), std::abs(right_speed));
  const uint8_t pwm = speedToPwm(max_mag);

  // 정지→움직임 전환: 킥스타트, 브레이크 취소
  if (last_speed_ == 0 && pwm > 0) {
    kickstart_time_ = this->now();
    is_braking_ = false;
    RCLCPP_INFO(this->get_logger(), "[KICKSTART] 시작 (pwm=%u → boost=%u for %dms)",
      pwm, KICKSTART_PWM_, KICKSTART_DURATION_MS_);
  }

  // 움직임→정지 전환: 브레이크 시작
  if (last_speed_ > 0 && pwm == 0) {
    brake_time_ = this->now();
    is_braking_ = true;
    brake_left_dir_  = (last_left_dir_  == kLeftFwd)  ? kLeftRev  :
                       (last_left_dir_  == kLeftRev)   ? kLeftFwd  : kDirStop;
    brake_right_dir_ = (last_right_dir_ == kRightFwd) ? kRightRev :
                       (last_right_dir_ == kRightRev)  ? kRightFwd : kDirStop;
    RCLCPP_INFO(this->get_logger(), "[BRAKE] 시작 (boost=%u for %dms)",
      BRAKE_PWM_, BRAKE_DURATION_MS_);
  }

  if (pwm == kSpeedStop) {
    last_left_dir_  = kDirStop;
    last_right_dir_ = kDirStop;
    last_speed_     = kSpeedStop;
  } else {
    last_left_dir_ = left_dir;
    last_right_dir_ = right_dir;
    last_speed_ = pwm;
  }

  last_cmd_time_ = this->now();
  has_cmd_ = true;
}

uint8_t WheelController::directionFromSpeed(double speed) const {
  if (std::abs(speed) <= deadband_) {
    return kDirStop;
  }
  return (speed >= 0.0) ? kLeftFwd : kLeftRev;
}

uint8_t WheelController::speedToPwm(double speed) const {
  const double mag = std::abs(speed);
  if (mag <= deadband_) {
    return 0;
  }
  const double clamped = std::min(mag, max_linear_speed_);
  const double ratio = (max_linear_speed_ <= 0.0) ? 0.0 : (clamped / max_linear_speed_);
  const int pwm = static_cast<int>(std::lround(ratio * 255.0));
  const int effective = (pwm > 0) ? std::max(pwm, min_pwm_) : 0;
  return static_cast<uint8_t>(std::clamp(effective, 0, 255));
}

void WheelController::sendMotorCommand(uint8_t speed, uint8_t left_dir, uint8_t right_dir) {
  if (serial_fd_ < 0) {
    return;
  }

  // 킥스타트: 정지→움직임 직후 KICKSTART_DURATION_MS_ 동안 부스트
  uint8_t effective_speed = speed;
  uint8_t effective_left  = left_dir;
  uint8_t effective_right = right_dir;
  
  if (speed > 0) {
    const int64_t elapsed_ms = (this->now() - kickstart_time_).nanoseconds() / 1000000LL;
    if (elapsed_ms < KICKSTART_DURATION_MS_) {
      effective_speed = std::max(speed, KICKSTART_PWM_);
    }
  } else if (is_braking_) {
    // 브레이크: 움직임→정지 직후 BRAKE_DURATION_MS_ 동안 역방향 PWM
    const int64_t elapsed_ms = (this->now() - brake_time_).nanoseconds() / 1000000LL;
    if (elapsed_ms < BRAKE_DURATION_MS_) {
      effective_speed = BRAKE_PWM_;
      effective_left  = brake_left_dir_;
      effective_right = brake_right_dir_;
    } else {
      is_braking_ = false;
    }
  }

  // 직전과 동일한 명령이면 하드웨어 전송 생략
  if (effective_speed == last_sent_speed_ &&
      effective_left  == last_sent_left_  &&
      effective_right == last_sent_right_) {
    return;
  }
  last_sent_speed_ = effective_speed;
  last_sent_left_  = effective_left;
  last_sent_right_ = effective_right;

  char buffer[64];
  int len;
  if (effective_speed == kSpeedStop) {
    RCLCPP_DEBUG(this->get_logger(), "[MOTOR] send=s (stop)");
    len = std::snprintf(buffer, sizeof(buffer), "s\r\n");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "[MOTOR] send=%u (raw=%u) L=%u R=%u",
      effective_speed, speed, effective_left, effective_right);
    len = std::snprintf(buffer, sizeof(buffer), "%u,%u,%u\r\n", effective_speed,
                        effective_left, effective_right);
  }
  if (len <= 0) {
    return;
  }

  const ssize_t written = ::write(serial_fd_, buffer, static_cast<size_t>(len));
  if (written < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Failed to write to serial port");
  }
}

void WheelController::readSerialResponses() {
  if (serial_fd_ < 0) {
    return;
  }

  char buffer[128];
  const ssize_t nread = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
  if (nread > 0) {
    buffer[nread] = '\0';
    RCLCPP_DEBUG(this->get_logger(), "MCU: %s", buffer);
  }
}

bool WheelController::openSerialPort() {
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    return false;
  }

  termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    closeSerialPort();
    return false;
  }

  cfsetospeed(&tty, toTermiosBaud(baud_rate_));
  cfsetispeed(&tty, toTermiosBaud(baud_rate_));

  tty.c_cflag = static_cast<tcflag_t>(tty.c_cflag | CLOCAL | CREAD);
  tty.c_cflag = static_cast<tcflag_t>(tty.c_cflag & ~CSIZE);
  tty.c_cflag = static_cast<tcflag_t>(tty.c_cflag | CS8);
  tty.c_cflag = static_cast<tcflag_t>(tty.c_cflag & ~PARENB);
  tty.c_cflag = static_cast<tcflag_t>(tty.c_cflag & ~CSTOPB);
  tty.c_cflag = static_cast<tcflag_t>(tty.c_cflag & ~CRTSCTS);

  tty.c_iflag = static_cast<tcflag_t>(tty.c_iflag & ~(IXON | IXOFF | IXANY));
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    closeSerialPort();
    return false;
  }

  return true;
}

void WheelController::closeSerialPort() {
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}
