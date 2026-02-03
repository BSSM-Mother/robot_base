#include "robot_base/wheel_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {
constexpr uint8_t kDirStop = 0;
constexpr uint8_t kDirFwd = 1;
constexpr uint8_t kDirRev = 2;

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
      serial_port_(this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0")),
      baud_rate_(this->declare_parameter<int>("baud_rate", 115200)),
      wheel_distance_(this->declare_parameter<double>("wheel_distance", 0.25)),
      max_linear_speed_(this->declare_parameter<double>("max_linear_speed", 0.5)),
      deadband_(this->declare_parameter<double>("deadband", 0.02)),
      send_rate_hz_(this->declare_parameter<double>("send_rate_hz", 20.0)),
      cmd_timeout_ms_(this->declare_parameter<int>("cmd_timeout_ms", 300)),
      serial_fd_(-1),
      last_cmd_time_(this->now()),
      has_cmd_(false),
      last_speed_(0),
      last_left_dir_(kDirStop),
      last_right_dir_(kDirStop) {
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
    last_speed_ = 0;
    last_left_dir_ = kDirStop;
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

  const uint8_t left_dir = directionFromSpeed(left_speed);
  const uint8_t right_dir = directionFromSpeed(right_speed);

  const double max_mag = std::max(std::abs(left_speed), std::abs(right_speed));
  const uint8_t pwm = speedToPwm(max_mag);

  if (pwm == 0) {
    last_left_dir_ = kDirStop;
    last_right_dir_ = kDirStop;
    last_speed_ = 0;
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
  return (speed >= 0.0) ? kDirFwd : kDirRev;
}

uint8_t WheelController::speedToPwm(double speed) const {
  const double mag = std::abs(speed);
  if (mag <= deadband_) {
    return 0;
  }
  const double clamped = std::min(mag, max_linear_speed_);
  const double ratio = (max_linear_speed_ <= 0.0) ? 0.0 : (clamped / max_linear_speed_);
  const int pwm = static_cast<int>(std::lround(ratio * 255.0));
  return static_cast<uint8_t>(std::clamp(pwm, 0, 255));
}

void WheelController::sendMotorCommand(uint8_t speed, uint8_t left_dir, uint8_t right_dir) {
  if (serial_fd_ < 0) {
    return;
  }

  char buffer[64];
  const int len = std::snprintf(buffer, sizeof(buffer), "%u,%u,%u\r\n", speed, left_dir,
                                right_dir);
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
