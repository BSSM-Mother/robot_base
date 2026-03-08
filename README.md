# robot_base - 모터 제어 드라이버

**로봇 바퀴 모터를 시리얼 통신으로 제어하는 하드웨어 인터페이스 패키지**

## 📌 개요

로봇의 바퀴 모터를 직렬(시리얼) 통신을 통해 제어합니다. ROS2 토픽에서 속도 명령을 받아 모터 드라이버에 직접 전달합니다.

## 🎯 주요 기능

- ✅ 시리얼 포트 통신 (기본값: 115200 bps)
- ✅ `cmd_vel_raw` 토픽 구독 → 모터 명령 변환
- ✅ 전진/후진/회전 제어
- ✅ 속도 제한 및 안전 기능

## 📡 토픽 및 파라미터

### 구독 토픽 (입력)
- **`cmd_vel`** (`geometry_msgs/Twist`)
  - `linear.x`: 선속도 (-0.8 ~ 0.8 m/s)
  - `angular.z`: 각속도 (-1.5 ~ 1.5 rad/s)

### 발행 토픽 (출력)
- 시리얼 패킷 → 모터 드라이버

## ⚙️ 파라미터 설정

기본 설정 (robot_launch 패키지의 설정 파일):
```yaml
wheel_controller:
  ros__parameters:
    port: "/dev/ttyUSB0"          # 시리얼 포트 (Linux) 또는 "COM3" (Windows)
    baudrate: 115200              # 바우드레이트
    motor_left_id: 1              # 좌측 모터 ID
    motor_right_id: 2             # 우측 모터 ID
```

## 💾 빌드

```bash
colcon build --packages-select robot_base
```

## 🚀 실행

```bash
ros2 run robot_base wheel_controller
```

또는 robot_launch를 통해 전체 시스템과 함께 실행:
```bash
ros2 launch robot_launch robot.launch.py
```

## 📊 속도 제어 프로토콜

시리얼 통신 패킷 형식:
```
[Header: 0xFF] [Motor_ID] [Direction] [Speed] [Checksum]

Direction: 0=정지, 1=전진, 2=후진
Speed: 0~255 (0=정지, 255=최고속)
```

## 🔧 하드웨어 연결

1. 모터 드라이버를 Raspberry Pi (또는 제어 컴퓨터)의 UART/USB 시리얼 포트에 연결
2. `/dev/ttyUSB0` 또는 해당 포트로 설정
3. 바우드레이트 115200 설정 확인

## ⚠️ 주의사항

- 시리얼 포트 권한 확인: `sudo chmod 666 /dev/ttyUSB0`
- 모터 전원 차단 시 로봇이 움직이지 않음
- 속도 명령 없으면 자동으로 정지

## 📝 소스 파일

- `src/wheel_controller.cpp`: 메인 제어 로직
- `include/robot_base/wheel_controller.hpp`: 헤더 파일

## 🔗 연관 패키지

- **입력 출처**: `robot_control` (제어 로직)
- **출력 대상**: 모터 드라이버 하드웨어
- **의존성**: rclcpp, geometry_msgs, sensor_msgs
